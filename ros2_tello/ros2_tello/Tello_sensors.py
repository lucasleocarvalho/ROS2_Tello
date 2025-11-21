from djitellopy import Tello
import threading
import time
import numpy as np
from transforms3d.euler import euler2quat
from rclpy import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState

class KalmanFilter():
    def __init__(self, dt):
        self.dt = dt
        self.x = np.zeros((6, 1))
        self.P = np.eye(6) * 100
        self.Q = np.eye(6) * 0.1
        self.R = np.eye(4) * 5.0

        self.F = np.array([
            [1, 0, self.dt, 0, 0.5*self.dt**2, 0],
            [0, 1, 0, self.dt, 0, 0.5*self.dt**2],
            [0, 0, 1, 0, self.dt, 0],
            [0, 0, 0, 1, 0, self.dt],
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
    

class TelloSensors():
    def __init__(self, tello):
        self.tello = tello
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

        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()


    def _loop(self):
        while self.running:
            vx, vy, self.vz = self.velocity()
            ax, ay, self.az = self.acceleration()

            self.battery = self.tello.get_battery()
            self.pose_z = self.tello.get_height()


            z = np.array([
                  [vx],
                  [vy],
                  [ax],
                  [ay]
            ])
            state = self.kalman.algorithm(z)

            self.pose_x = state[0, 0]
            self.pose_y = state[1, 0]
            self.vx = state[2, 0]
            self.vy = state[3, 0]
            self.ax = state[4, 0]
            self.ay = state[5, 0]

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
        vx = self.tello.get_speed_x() * 10  # frente
        vy = self.tello.get_speed_y() * 10  # direita
        vz = self.tello.get_speed_z() * 10

        return vx, vy, vz


    def acceleration(self):
        ax = self.tello.get_acceleration_x()
        ay = self.tello.get_acceleration_y()
        az = self.tello.get_acceleration_z()

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
    

class ROS2TelloSensors(Node):
    def __init__(self):
        super().__init__('ros2_tello_sensors')
        self.sensors = TelloSensors()
        self.time = 0.05

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)

        self.odom_publish = self.create_publisher(Odometry, '/tello/odom', qos)
        self.battery_publish = self.create_publisher(BatteryState, '/tello/battery', qos)
        self.timer = self.create_timer(self.time, self.timer_callback)
    
    def odom_callback(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        x, y, z = self.sensors.pose_callback()
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z

        qx, qy, qz, w = self.sensors.euler_to_quaternion()
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = w

        vx, vy, vz = self.sensors.velocity_callback()
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
        battery.percentage = self.sensors.battery_callback() / 100

        self.battery_publish.publish(battery)
    
    def timer_callback(self):
        self.odom_callback()
        self.battery_callback()