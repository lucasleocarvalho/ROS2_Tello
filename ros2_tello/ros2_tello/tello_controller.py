import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import math

class TelloController(Node):
    def __init__(self):
        super().__init__('tello_controller')
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
        vy = 0
        vz = 0
        if self.x - 10 <= 100 <= self.x + 10:
            self.vvx = 0
        
        if self.y - 10 <= 100 <= self.y + 10:
            vy = 0
        
        if self.z - 10 <= 100 <= self.z + 10:
            vz = 0
        
        if self.vvx == 0 and vy == 0 and vz == 0:
            self.request_land()
        
        
        self.cmd_vel_callback(self.vvx, 0.0, 0.0, 0.0)


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