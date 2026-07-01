# ROS2_Tello
## Sobre o projeto 
O projeto ROS2_Tello foi feito baseado no Tello SDK 3.0. Funcionalidades incluem odometria, movimentação e conexão ao computador via wifi.
## Iniciando
O projeto foi testado no Ubuntu 22.04 com ROS2 Humble.
### Pré requisitos
- ROS2 Humble 
- Python3
- Numpy
- Rclpy
## Instalação
Para instalar ROS2_Tello:
```bash
mkdir tello_ws && cd tello_ws
mkdir src && cd src
git clone https://github.com/lucasleocarvalho/ROS2_Tello/tree/Erick/ROS2_Tello.git
cd ..
colcon build
source install/setup.bash
```

## Uso
Conecte o computador ao wifi gerado pelo Tello.

Para conectar o drone: 
```bash
ros2 run ros2_tello tello_ros
``` 

Para iniciar o controle do drone:
``` bash
ros2 run ros2_tello tello_controller 
```
Para decolar o drone:
``` bash 
ros2 service call /tello/takeoff std_srvs/srv/Empty 
```
Para pousar o drone:
``` bash
ros2 service call /tello/land std_srvs/srv/Empty 
```
Para movimentar o drone:
``` bash
ros2 topic pub /tello/cmd_vel -1 geometry_msgs/msg/Twist "{linear: {x: 0.0,  y: 0.0,  z: 0.0} , angular: {x: 0.0, y: 0.0, z: 0.0}}"
```
Onde as velocidades lineares e angulares são dadas em cm/s e tem range de -100 a 100
