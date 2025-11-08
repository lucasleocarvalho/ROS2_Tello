from djitellopy import Tello
import threading
import time

class TelloSensors():
    def __init__(self, tello):
        self.tello = tello

        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_z = 0.0

        self.last_time = time.time()

        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()


    def _loop(self):
        while self.running:
            now = time.time()
            dt = now - self.last_time
            self.last_time = now

            vx, vy, vz = self.velocity()


            self.pose_x += vx * dt
            self.pose_y += vy * dt
            self.pose_z += vz * dt

            time.sleep(0.05)   


    def distance(self):
        '''Return the position of the drone [x, y, z]'''
        return self.pose_x, self.pose_y, self.pose_z


    def velocity(self):
        vx = self.tello.get_speed_x() * 10
        vy = self.tello.get_speed_y() * 10
        vz = self.tello.get_speed_z() * 10
        return vx, vy, vz


    def acceleration(self):
        ax = self.tello.get_acceleration_x()
        ay = self.tello.get_acceleration_y()
        az = self.tello.get_acceleration_z()
        return ax, ay, az

    def orientation(self):
        row = self.tello.get_row()
        pitch = self.tello.get_pitch()
        yaw = self.tello.get_yaw()

        return row, pitch, yaw
        

    def stop(self):
        self.running = False
