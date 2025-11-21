import time
import Tello_sensors
from djitellopy import Tello
import math

class TelloController:
    def __init__(self, tello):
        #self.tello = Tello()
        self.tello = tello
        self.sensors = Tello_sensors.TelloSensors(tello)

    def yaw_speed(self, vx, vy):
        if vx == 0 and vy == 0:
            return 0
        
        _, _, yaw = self.sensors.orientation()
        yaw_target = math.degrees(math.atan2(vy, vx))
        yaw_error = (yaw_target - yaw + 180) % 360 - 180

        kp = 1.0
        yaw_speed = kp * yaw_error
        yaw_speed = max(min(yaw_speed, 100), -100)

        return yaw_speed

        
    
    def pos_ctrl(self, x: int, y: int, z: int, yaw: int, vx: int, vy: int, vz: int, yaw_speed: int, threshold):
        
        while True:
            pose_x, pose_y, pose_z = self.sensors.distance()
            roll, pitch, yaw_mea = self.sensors.orientation()
            #yaw_speed = self.yaw_speed(vx, vy)
            
            if x - threshold <= abs(pose_x) <= x + threshold:
                vx = 0
            if y - threshold <= abs(pose_y) <= y + threshold:
                vy = 0
            if z - threshold <= abs(pose_z) <= z + threshold:
                vz = 0
            
            if yaw - threshold <= abs(yaw_mea) <= yaw + threshold:
                yaw_speed = 0
            
            
            #if vx == 0 and vy == 0:
                #yaw_speed = 0
            

            self.tello.send_rc_control(int(vy), int(vx), int(vz), yaw_speed)
            px, py, pz = self.sensors.distance()

            vvx, vvy, vvz = self.sensors.velocity()
            ax, ay, az = self.sensors.acceleration()

            print(f"X: {px} \nY: {py} \nZ: {pz} \nYaw: {yaw_mea} \nVX: {vvx} \nVY: {vvy} \nAX: {ax} \nAY: {ay}")

            if vx == 0 and vy == 0 and vz == 0 and yaw_speed == 0:
                break

            time.sleep(0.05)