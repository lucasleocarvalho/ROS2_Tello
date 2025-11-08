from djitellopy import Tello
import time
import Tello_sensors

tello = Tello()
tello.connect()

sensors = Tello_sensors.Sensors(tello)
#sensors.acceleration()

user = input()

if user == "1":
    tello.takeoff()
    tello.move_forward(100)
    
    while True:
        x, y, z = sensors.distance()
        print(x)
        if 0.85 <= x <= 1.15:
            break
    tello.land()
    
