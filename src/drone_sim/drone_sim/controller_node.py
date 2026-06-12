import rclpy
from rclpy.node import Node

from drone_sim_msgs.msg import DroneState, DroneCmd

from drone_sim.controller import WaypointController
from drone_sim.env import Waypoint
from drone_sim.constants import DT

class DroneStateView():
    def __init__(self, msg: DroneState):
        self.position = list(msg.gps_position)
        self.velocity = list(msg.velocity)

class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')
        
        self._log_counter = 0
        
        self.last_state = None

        self.controller = WaypointController()
        
        self.current_waypoint = Waypoint(0.0, 0.0, 3.0, 'PlaceHolder - takeoff')

        # Recebe os sensores do physics_node
        self.sub = self.create_subscription(DroneState, '/drone_state', self.drone_callback, 10)

        # Publica comandos de thrust/roll/pitch/yaw
        self.pub = self.create_publisher(DroneCmd, '/cmd_drone', 10)

        # Mesmo Timer para correto funcionamento do PID em cascata
        self.timer = self.create_timer(DT, self.calculateCmd)

        self.get_logger().info("Controller Node Iniciado")

    def drone_callback(self, msg: DroneState):
        self.last_state = msg

    def calculateCmd(self):
        if self.last_state is None:
            # Ainda não recebemos nenhum estado do physics_node
            return

        drone_view = DroneStateView(self.last_state)
        thrust, roll_cmd, pitch_cmd, yaw_cmd = self.controller.calculateCommands(
                                            drone_view, self.current_waypoint, DT)
        msg = DroneCmd()
        msg.thrust = thrust
        msg.roll_cmd = roll_cmd
        msg.pitch_cmd = pitch_cmd
        msg.yaw_cmd = yaw_cmd
        
        self.pub.publish(msg)
        
        # --- DEBUG LOG (remover após validação) ---
        self._log_counter += 1
        if self._log_counter >= int(1.0 / DT):
            self._log_counter = 0
            self.get_logger().info(
                f"[CONTROLLER] thrust={thrust:.2f} "
                f"roll_cmd={roll_cmd:.3f} pitch_cmd={pitch_cmd:.3f} "
                f"target=({self.current_waypoint.x:.1f}, {self.current_waypoint.y:.1f}, {self.current_waypoint.z:.1f})"
            )
        # --- FIM DEBUG LOG ---


def main(args=None):
    rclpy.init(args=args)
    controller = ControllerNode()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()