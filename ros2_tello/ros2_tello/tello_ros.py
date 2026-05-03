from djitellopy import Tello
import logging
import threading
import time
import numpy as np
from transforms3d.euler import euler2quat
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import csv, os, json
from ament_index_python.packages import get_package_share_directory

class KalmanFilter():
    def __init__(self, dt):
        self.dt = dt
        self.x = np.zeros((6, 1))
        self.P = np.eye(6) * 100
        self.R = np.diag([1, 10.0, 10.0, 10.0])  # z, vx, vy, vz, ax, ay, vz
        self.Q = np.diag([0.1, 0.1, 0.1, 150, 150, 150])


        self.F = np.array([
            [1, 0, 0, self.dt, 0, 0],
            [0, 1, 0, 0, self.dt, 0],
            [0, 0, 1, 0, 0, self.dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
        self.H = np.array([
            [0, 0, 1, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])

    def algorithm(self, z):
        x_next = self.F @ self.x
        P_next = self.F @ self.P @ self.F.T + self.Q

        K = P_next @ self.H.T @ np.linalg.inv(self.H @ P_next @ self.H.T + self.R)
        self.x = x_next + K @ (z - self.H @ x_next)   
        self.P = (np.eye(6) - K @ self.H) @ P_next

        return self.x
    

class TelloROS():
    def __init__(self):
        self.tello = Tello()
        self.kalman = KalmanFilter(0.05)

        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_z = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0

        self.battery = 0.0

        self.last_yaw = 0.0

        self.last_height = 0.0


    def connect(self):
        self.tello.connect()
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        return

    def _loop(self):
        while self.running:
            vx, vy, vz = self.velocity()
            #ax, ay, az = self.acceleration()

            self.battery = self.tello.get_battery()
            
            alpha = 0.3

            height_raw = self.tello.get_height()

            if height_raw == 0:
                height = self.last_height
            else:
                height = alpha * height_raw + (1 - alpha) * self.last_height
                self.last_height = height

            z = np.array([
                  [self.last_height / 100],
                  [vx],
                  [vy],
                  [vz]
            ])
            state = self.kalman.algorithm(z)

            self.pose_x = state[0, 0]
            self.pose_y = state[1, 0]
            self.pose_z = state[2, 0]
            #self.pose_z = self.last_height / 100
            self.vx = state[3, 0]
            self.vy = state[4, 0]
            self.vz = state[5, 0]

            if self.pose_z < 0.0:
                self.pose_z = 0.0

            time.sleep(0.05)

    def distance(self):
        '''Return the position of the drone [x, y, z]'''
        return self.pose_x, self.pose_y, self.pose_z

    def rotation(self, x_body, y_body):
        # pegar yaw
        _, _, yaw = self.orientation()
        yaw_rad = np.deg2rad(yaw)

        # matriz de rotação
        R = np.array([
            [np.cos(yaw_rad), -np.sin(yaw_rad)],
            [np.sin(yaw_rad),  np.cos(yaw_rad)]
        ])

        xy_global = R @ np.array([[x_body], [y_body]])

        x = xy_global[0, 0]
        y = xy_global[1, 0]

        return x, y

    def velocity(self):
        vx = self.tello.get_speed_x() / 10  # frente
        vy = self.tello.get_speed_y() / 10  # direita
        vz = self.tello.get_speed_z() / 10

        return vx, vy, vz

    def acceleration(self):
        ax = self.tello.get_acceleration_x() / 100
        ay = self.tello.get_acceleration_y() / 100
        az = self.tello.get_acceleration_z() / 100

        ax, ay = self.rotation(ax, ay)

        return ax, ay, az

    def orientation(self):
        row = self.tello.get_roll()
        pitch = self.tello.get_pitch()
        yaw = self.tello.get_yaw()

        return row, pitch, yaw

    def stop(self):
        self.running = False
    
    def pose_callback(self):
        return self.pose_x, self.pose_y, self.pose_z
    
    def velocity_callback(self):
        return self.vx, self.vy, self.vz
    
    def euler_to_quaternion(self):
        roll, pitch, yaw = self.orientation()
        roll = np.deg2rad(roll)
        pitch = np.deg2rad(pitch)
        yaw = np.deg2rad(yaw)

        yaw_unwrapped = np.unwrap([self.last_yaw, yaw])[1]
        self.last_yaw = yaw_unwrapped

        w, qx, qy, qz = euler2quat(roll, pitch,yaw_unwrapped, axes='rxyz')
        return qx, qy, qz, w
    
    def battery_callback(self):
        return self.battery
    
    def takeoff(self):
        self.tello.takeoff()
    
    def land(self):
        self.tello.land()

    def move(self, vx, vy, vz, vw):
        self.tello.send_rc_control(int(vy), int(vx), int(vz), int(vw))

class ROS2TelloSensors(Node):
    def __init__(self):
        super().__init__('ros2_tello_sensors')
        logging.getLogger('djitellopy').setLevel(logging.CRITICAL)

        share_dir = get_package_share_directory('ros2_tello')
        wp_path = os.path.join(share_dir, 'config', 'waypoints.json')
        with open(wp_path, 'r') as f:
            self.waypoints = json.load(f)

        self.csv_file = open('ground_truth.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp',
            'x',
            'y',
            'z',
            'vx',
            'vy',
            'vz'
        ])
        
        self.index = 0      

        self.tello_ros = TelloROS()
        self.time = 0.05
        self.flying = False
        self.goal = False

        while True:
            try:
                self.get_logger().info("Tentando conectar ao Tello...")
                self.tello_ros.connect()
                self.get_logger().info("Conexão com Tello bem-sucedida!")
                break
            except Exception as e:
                self.get_logger().warn(f"Falha ao conectar ao Tello: {e}. Tentando novamente...")
                time.sleep(1)
        
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        qos2 = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)

        self.odom_publish = self.create_publisher(Odometry, '/tello/odom', qos)
        self.battery_publish = self.create_publisher(BatteryState, '/tello/battery', qos2)
        
        self.cmd_vel = self.create_subscription(Twist, '/tello/cmd_vel', self.twist, qos2)

        self.takeoff_srv = self.create_service(Empty, '/tello/takeoff', self.takeoff)
        self.land_srv = self.create_service(Empty, '/tello/land', self.land)
        
        self.timer = self.create_timer(self.time, self.timer_callback)
    
    def odom_callback(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        x, y, z = self.tello_ros.pose_callback()
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = float(z)

        qx, qy, qz, w = self.tello_ros.euler_to_quaternion()
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = w

        vx, vy, vz = self.tello_ros.velocity_callback()
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0

        self.odom_publish.publish(odom)
    

    def battery_callback(self):
        battery = BatteryState()
        battery.header.stamp = self.get_clock().now().to_msg()
        battery.percentage = self.tello_ros.battery_callback() / 100

        self.battery_publish.publish(battery)

    def takeoff(self, request, response):
        self.tello_ros.takeoff()
        self.get_logger().info("Decolando...")
        return response
    
    def land(self, request, response):
        self.tello_ros.land()
        self.get_logger().info("Pousando...")
        return response

    def twist(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        vz = msg.linear.z
        vw = msg.angular.z

        self.tello_ros.move(vx, vy, vz, vw)

    def timer_callback(self):
        self.odom_callback()
        self.battery_callback()
        self.move_manually()
    
    def move_manually(self):
        x, y, z = self.tello_ros.pose_callback()
        vvx, vvy, vvz = self.tello_ros.velocity_callback()

        print(f"X: {x:.2f}   Y: {y:.2f}  Z: {z:.2f} ")
        print(f"VX: {vvx:.2f}  VY: {vvy:.2f}  Vz: {vvz:.2f}")

        timestamp = self.get_clock().now().nanoseconds / 1e-9
        self.csv_writer.writerow([
            timestamp,
            x,
            y,
            z,
            vvx,
            vvy,
            vvz
        ])    

        if self.goal is not True:
            battery = self.tello_ros.battery_callback()
            if battery <= 15:
                self.get_logger().warning(f"Bateria em {battery:.0f}%. O drone não irá decolar...")
                time.sleep(0.5)
                return
            
            try:
                if self.index >= len(self.waypoints):
                    self.get_logger().info("Todos os waypoints concluídos! Iniciando Pouso...")
                    self.flying = False
                    self.goal = True
                    self.tello_ros.land()
                    return

                wp = self.waypoints[self.index]

                if self.flying == False and self.index == 0:
                    #self.get_logger().info(f"Bateria atual: {battery:.0f}%. Decolando...")
                    self.tello_ros.takeoff()
                    self.flying = True

                vx = wp["vx"]
                vy = wp["vy"]
                vz = wp["vz"]
                vw = 0

                x_goal = wp["x"] 
                y_goal = wp["y"] 
                z_goal = wp["z"] 
                yaw_goal = 0.0

                treshold = 0.2

                if abs(x - x_goal) <= treshold:
                    vx = 0
                if abs(y - y_goal) <= treshold:
                    vy = 0
                if abs(z - z_goal) <= treshold:
                    vz = 0

                if vx != 0 or vy != 0 or vz != 0:
                    self.tello_ros.move(vx, vy, vz, vw)

                if vx == 0 and vy == 0 and vz == 0 and self.flying is True:
                    self.index += 1
                    

            except:
                if self.flying is True:
                    self.get_logger().warning("Pouso emergencial acionado...")
                    self.tello_ros.land()



def main(args=None):
    rclpy.init(args=args)

    try:
        ros2_tello_sensors = ROS2TelloSensors()
        rclpy.spin(ros2_tello_sensors)
        
    except KeyboardInterrupt:
        ros2_tello_sensors.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()