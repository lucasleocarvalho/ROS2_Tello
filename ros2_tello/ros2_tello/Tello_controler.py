import time
import Tello_sensors
from djitellopy import Tello
import math

class TelloController:
    def __init__(self):
        self.tello = Tello()
        self.sensors = Tello_sensors.TelloSensors()

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

        
    
    def pos_ctrl(self, x: int, y: int, z: int, vx: int, vy: int, vz: int, threshold):

        """
        Controla o Tello usando controle de posição aproximado baseado em velocidades.

        Parâmetros:
            x (int): Posição desejada no eixo X em metros aproximados.
            y (int): Posição desejada no eixo Y em metros aproximados.
            z (int): Altura desejada no eixo Z em metros aproximados.
            vx (int): Velocidade inicial no eixo X (m/s).
            vy (int): Velocidade inicial no eixo Y (m/s).
            vz (int): Velocidade inicial no eixo Z (m/s).
            threshold (float): Raio de tolerância em metros ao redor do destino.

        Funcionamento:
            Usa as velocidades e a posição estimada pelos sensores para mover o drone
            em direção ao ponto desejado. As velocidades são zeradas automaticamente
            quando o drone entra na região de tolerância.

        Retorno:
            None
        """
        
        while True:
            pose_x, pose_y, pose_z = self.sensors.distance()
            yaw_speed = self.yaw_speed(vx, vy)
            
            if x - threshold <= pose_x <= x + threshold:
                vx = 0
            if y - threshold <= pose_y <= y + threshold:
                vy = 0
            if z - threshold <= pose_z <= z + threshold:
                vz = 0
            
            if vx == 0 and vy == 0:
                yaw_speed = 0


            self.tello.send_rc_control(int(vx), int(vy), int(vz), int(yaw_speed))

            if vx == 0 and vy == 0 and vz == 0:
                break

            time.sleep(0.05)