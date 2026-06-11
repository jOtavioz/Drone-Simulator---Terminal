import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from drone_sim.drone import Drone
from drone_sim.physics import Physics
from drone_sim.constants import DT

class PhysicsNode(Node):

    def __init__(self):

        super().__init__('physics_node')

        self.drone = Drone()

        self.pub = self.create_publisher(String, '/drone_state', 10)
        
        self.sub = self.create_subscription(String, '/cmd_drone', self.controller_callback, 10)

        self.sim_time = 0.0

        self.timer = self.create_timer(DT,self.update_physics)

        self.get_logger().info("Physics Node Iniciado")

    def update_physics(self):                       # funcão de callback executada pelo timer a cada DT 

        Physics.updatePhysics(self.drone,DT)
        self.sim_time += DT

        msg = String()
        
        msg.data = (
            f't={self.sim_time:.2f}s, '
            f'x={self.drone.position[0]:.2f}, '
            f'y={self.drone.position[1]:.2f}, '
            f'z={self.drone.position[2]:.2f}'
        )

        self.pub.publish(msg)
        self.get_logger().info(f'Physics Update: \"{msg.data}\"')
    
    def controller_callback(self, msg):
        self.get_logger().info(f'Cmd Heard \"{msg.data}\"')


def main(args=None):

    rclpy.init(args=args)

    node = PhysicsNode()

    rclpy.spin(node)        # roda o nó "eternamente" esperando por eventos do timer

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()