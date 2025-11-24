import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from djitellopy import Tello

class ModoController(Node):
    def __init__(self):
        super().__init__('modo_controller')
        
        # Inicializar conexão com o drone via djitellopy
        self.tello = Tello()
        self.tello.connect()
        self.tello.streamon()  # Ativar stream de vídeo (opcional, mas útil para debug)
        
        # Modo de operação: False = padrão (sem limitador), True = limitado (sem yaw)
        self.modo_limitado = False
        
        # Subscriber para comandos de velocidade (Twist)
        self.subscription = self.create_subscription(
            Twist,
            '/tello/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Serviço para alternar modo (ativado pelo usuário)
        self.srv = self.create_service(SetBool, '/tello/toggle_modo', self.toggle_modo_callback)
        
        self.get_logger().info('ModoController inicializado. Modo padrão ativo. Use /tello/toggle_modo para alternar.')

    def toggle_modo_callback(self, request, response):
        self.modo_limitado = request.data
        if self.modo_limitado:
            self.get_logger().info('Modo limitado ativado: rotações angulares bloqueadas.')
        else:
            self.get_logger().info('Modo padrão ativado: movimentos livres permitidos.')
        response.success = True
        response.message = 'Modo alterado com sucesso.'
        return response

    def cmd_vel_callback(self, msg):
        # Aplicar limitações se modo limitado estiver ativo
        if self.modo_limitado:
            msg.angular.z = 0.0  # Bloquear yaw (rotação angular no plano XY)
            self.get_logger().debug('Modo limitado: yaw zerado.')
        
        # Enviar comandos para o drone via djitellopy
        # Converter Twist para comandos Tello (baseado no SDK: rc a b c d para roll/pitch/throttle/yaw)
        # Assumindo escala padrão: linear.x/y/z para frente/trás/lados/cima/baixo, angular.z para yaw
        a = int(msg.linear.y * 100)  # Roll (esquerda/direita) - limitado a -100/100
        b = int(msg.linear.x * 100)  # Pitch (frente/trás)
        c = int(msg.linear.z * 100)  # Throttle (cima/baixo)
        d = int(msg.angular.z * 100)  # Yaw (rotação), zerado se limitado
        
        # Enviar comando RC
        self.tello.send_rc_control(a, b, c, d)
        self.get_logger().debug(f'Comando enviado: rc {a} {b} {c} {d}')

def main(args=None):
    rclpy.init(args=args)
    node = ModoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.tello.streamoff()
        node.tello.end()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
