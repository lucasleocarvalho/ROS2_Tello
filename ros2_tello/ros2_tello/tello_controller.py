import time
import json
import csv
import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import Empty

class TelloController(Node):
    def __init__(self):
        super().__init__('tello_controller')

        share_dir = get_package_share_directory('ros2_tello')
        wp_path = os.path.join(share_dir, 'config', 'waypoints.json')
        with open(wp_path, 'r') as f:
            self.waypoints = json.load(f)

        self.csv_file = open('ground_truth.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'time',
            'x',
            'y',
            'z',
            'vx',
            'vy',
            'vz'
        ])
        
        self.index = 0
        self.tolerance = 5

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        qos2 = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)

        timer = 0.05
        self.flying = False

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.cmd_vel = self.create_publisher(Twist, '/tello/cmd_vel', qos2)
        self.path_publisher = self.create_publisher(Path, '/tello/path', qos)

        self.odom = self.create_subscription(Odometry, '/tello/odom', self.odometry_callback, qos)
        self.battery = self.create_subscription(BatteryState, '/tello/battery', self.battery_callback, qos2)

        self.takeoff_cli = self.create_client(Empty, '/tello/takeoff')
        self.land_cli = self.create_client(Empty, '/tello/land')
        self.request = Empty.Request()

        self.path = Path()
        self.path.header.frame_id = 'map' 

        while not self.takeoff_cli.wait_for_service(1.0):
            self.get_logger().info("Aguardando serviço /tello/takeoff...")

        while not self.land_cli.wait_for_service(1.0):
            self.get_logger().info("Aguardando serviço /tello/land...")
        

        self.timer = self.create_timer(timer, self.timer_callback)
    
    def odometry_callback(self, msg):
        #print("Recebendo odometria")
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.vz = msg.twist.twist.linear.z

        pose = PoseStamped()

        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.path.header = msg.header
        self.path.poses.append(pose)

        self.path_publisher.publish(self.path)

        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.csv_writer.writerow([
            timestamp,
            self.x,
            self.y,
            self.z,
            self.vx,
            self.vy,
            self.vz
        ])
        

    def battery_callback(self, msg):
        return msg.current
    
    def request_takeoff(self):
        if self.flying != False:
            return
        
        future = self.takeoff_cli.call_async(self.request)
        #rclpy.spin_until_future_complete(self, future)
        self.flying = True
        return future.result()
    
    def request_land(self):
        if self.flying != True:
            return
        
        future = self.land_cli.call_async(self.request)
        #rclpy.spin_until_future_complete(self, future)
        self.flying = False
        return future.result()

    def cmd_vel_callback(self, vx, vy, vz, vw):
        msg = Twist()

        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = float(vz)

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(vw)
        
        self.cmd_vel.publish(msg)

    def timer_callback(self):

        if self.index >= len(self.waypoints):
            self.get_logger().info("Todos os waypoints concluídos! Pousando...")
            self.request_land()
            return
        
        wp = self.waypoints[self.index]

        if self.index == 0 and self.flying == False:
            self.request_takeoff()
            self.get_logger().info("Iniciando voo...")
            return

        target_x = wp["x"]
        target_y = wp["y"]
        target_z = wp["z"]
        #target_yaw = wp["yaw"]

        vx = wp["vx"]
        vy = wp["vy"]
        vz = wp["vz"]
        #vw = wp["vw"]

        if self.x - self.tolerance <= target_x <= self.x + self.tolerance:
            vx = 0
        
        if self.y - self.tolerance <= target_y <= self.y + self.tolerance:
            vy = 0
        
        if self.z - self.tolerance <= target_z <= self.z + self.tolerance:
            vz = 0
        
        if vx == 0 and vy == 0 and vz == 0:
            self.index += 1
            return

        self.cmd_vel_callback(vx, vy, vz, 0.0)


def main(args=None):
    rclpy.init(args=args)
    tello_controller = TelloController()

    try:
        rclpy.spin(tello_controller)

    except KeyboardInterrupt:
        tello_controller.get_logger().warn("Interrupção pelo usuário. Pousando...")
        tello_controller.request_land()

    except Exception as e:
        tello_controller.get_logger().error(f"Erro inesperado: {e}")
        tello_controller.request_land()

    finally:
        tello_controller.csv_file.close()
        tello_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()