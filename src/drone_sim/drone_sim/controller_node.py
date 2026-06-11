import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')

        # Cria o subscription para o drone_state (topico em que physics_node publica)
        self.sub = self.create_subscription(String, '/drone_state', self.drone_callback,10)
        
        #Cria o publisher para o /cmd_drone (topico que controller publica att de PID, thrust, roll, pitch e yaw)
        self.pub = self.create_publisher(String, '/cmd_drone', 10)
        
        self.timer = self.create_timer(1.0,self.calculateCmd)
        
        self.get_logger().info("Controller Node Iniciado")

    def drone_callback(self, msg):
        self.get_logger().info(f'Recebido: \"{msg.data}\"')
    
    def calculateCmd(self):                       # funcão de callback executada pelo timer a cada DT 

        msg = String()
        msg.data = "teste --> thrust = 50"

        self.pub.publish(msg)
        self.get_logger().info(f'Cmd Published: \"{msg.data}\"')


def main(args=None):
    rclpy.init(args=args)
    controller = ControllerNode()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()