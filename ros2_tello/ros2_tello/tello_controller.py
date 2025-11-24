import time
import json
import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

class TelloController(Node):
    def __init__(self):
        super().__init__('tello_controller')

        share_dir = get_package_share_directory('ros2_tello')
        wp_path = os.path.join(share_dir, 'config', 'waypoints.json')
        with open(wp_path, 'r') as f:
            self.waypoints = json.load(f)
        
        self.index = 0
        self.tolerance = 10

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        timer = 0.05
        self.flying = False

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.cmd_vel = self.create_publisher(Twist, '/tello/cmd_vel', qos)

        self.odom = self.create_subscription(Odometry, '/tello/odom', self.odometry_callback, qos)
        self.battery = self.create_subscription(BatteryState, '/tello/battery', self.battery_callback, qos)

        self.takeoff_cli = self.create_client(Empty, '/tello/takeoff')
        self.land_cli = self.create_client(Empty, '/tello/land')
        self.request = Empty.Request()

        while not self.takeoff_cli.wait_for_service(1.0):
            self.get_logger().info("Aguardando serviço /tello/takeoff...")

        while not self.land_cli.wait_for_service(1.0):
            self.get_logger().info("Aguardando serviço /tello/land...")
        

        self.timer = self.create_timer(timer, self.timer_callback)
    
    def odometry_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.vz = msg.twist.twist.linear.z
        

    def battery_callback(self, msg):
        return msg.current
    
    def request_takeoff(self):
        future = self.takeoff_cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        self.flying = True
        return future.result()
    
    def request_land(self):
        future = self.land_cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        self.flying = False
        return future.result()

    def cmd_vel_callback(self, vx, vy, vz, vw):
        msg = Twist()

        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = vw
        
        self.cmd_vel.publish(msg)

    def timer_callback(self):
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

        if self.index >= len(self.waypoints):
            self.get_logger().info("Todos os waypoints concluídos! Pousando...")
            self.request_land()
            return



def main(args=None):
    rclpy.init(args=args)
    tello_controller = TelloController()

    try:
        rclpy.spin(tello_controller)
    
    except KeyboardInterrupt:
        tello_controller.request_land()
        tello_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()