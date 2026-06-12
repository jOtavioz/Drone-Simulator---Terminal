import rclpy
from rclpy.node import Node

from drone_sim_msgs.msg import DroneState, DroneCmd

from drone_sim.drone import Drone
from drone_sim.physics import Physics
from drone_sim.env import Mission, obs1, obs2, wall
from drone_sim.constants import DT


class PhysicsNode(Node):

    def __init__(self):
        super().__init__('physics_node')
        
        self._log_counter = 0
        
        self.drone = Drone()

        self.mission = Mission()
        self.mission.addObstacle(obs1)
        self.mission.addObstacle(obs2)
        self.mission.addObstacle(wall)

        self.sim_time = 0.0

        # Publica o estado dos sensores (GPS, LIDAR, IMU)
        self.pub = self.create_publisher(DroneState, '/drone_state', 10)

        # Recebe comandos de thrust/roll/pitch/yaw do controller_node
        self.sub = self.create_subscription(DroneCmd, '/cmd_drone', self.cmd_callback, 10)

        self.timer = self.create_timer(DT, self.update_physics)

        self.get_logger().info("Physics Node Iniciado")

    def cmd_callback(self, msg: DroneCmd):
        # Aplica o comando recebido nos motores via mixer
        self.drone.mixer.mix_thrust(
            msg.thrust,
            msg.roll_cmd,
            msg.pitch_cmd,
            msg.yaw_cmd
        )

    def update_physics(self):
        Physics.updatePhysics(self.drone, DT)
        self.drone.updateSensors(self.mission)
        self.sim_time += DT

        msg = DroneState()
        msg.sim_time = self.sim_time

        gps = self.drone.gps.read
        msg.gps_position = [gps[0], gps[1], gps[2]]

        msg.lidar_altitude = self.drone.lidar.read

        msg.attitude = list(self.drone.attitude)
        msg.angular_velocity = list(self.drone.angularVelocity)

        msg.velocity = list(self.drone.velocity)

        self.pub.publish(msg)
        
         # --- DEBUG LOG (remover após validação) ---
        self._log_counter += 1
        if self._log_counter >= int(1.0 / DT):
            self._log_counter = 0
            self.get_logger().info(
                f"[PHYSICS] t={self.sim_time:.2f}s "
                f"pos=({self.drone.position[0]:.2f}, {self.drone.position[1]:.2f}, {self.drone.position[2]:.2f}) "
                f"vel=({self.drone.velocity[0]:.2f}, {self.drone.velocity[1]:.2f}, {self.drone.velocity[2]:.2f}) "
                f"att=({self.drone.attitude[0]:.3f}, {self.drone.attitude[1]:.3f}) "
                f"lidar={self.drone.lidar.read:.2f}"
            )
        # --- FIM DEBUG LOG ---


def main(args=None):
    rclpy.init(args=args)
    node = PhysicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()