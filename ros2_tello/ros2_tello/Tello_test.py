from djitellopy import Tello
import time
import tello_sensors
from Tello_controler import TelloController     #vx sempre para frente
                                                #vy sempe para os lados

tello = Tello()
tello.connect()

sensors = tello_sensors.TelloSensors(tello)
controller = TelloController(tello)
#sensors.acceleration()

ba = tello.get_battery()
print(f"Battery Level: {ba}")


user = input()



try:

    if user == "1":
        tello.takeoff()
        
        controller.pos_ctrl(x=100, y=0, z=0, yaw=0, vx=25, vy=0, vz=0, yaw_speed=0, threshold=10)
        controller.pos_ctrl(x=0, y=0, z=0, yaw=180, vx=0, vy=0, vz=0, yaw_speed=25, threshold=2)
        controller.pos_ctrl(x=0, y=0, z=0, yaw=0, vx=25, vy=0, vz=0, yaw_speed=0, threshold=10)
        tello.land()


except KeyboardInterrupt:
    tello.land()
