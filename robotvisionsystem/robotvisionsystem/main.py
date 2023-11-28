import rclpy
from rclpy.node import Node
from robotvisionsystem.RobotVisionSystem import RobotVisionSystem
from robotvisionsystem.Logger import Logger
from robotvisionsystem.Sensor import Sensor
# cmd_vel
from geometry_msgs.msg import Twist


class Core(Node):
    def __init__(self):
        super().__init__('robot_vistion_system_node')
        self.logger = Logger(self)
        self.timer = self.create_timer(0.1, self.control)
        self.robot_vision_system = RobotVisionSystem(self)
        self.logger.info("robot_vistion_system_node start")

    def control(self):
        if self.robot_vision_system.sensor.init() == False:
            self.logger.info("robot_vistion_system_node sensor wait...")
            return
        self.robot_vision_system.control()


def main(args=None):
    rclpy.init(args=args)
    core = Core()
    rclpy.spin(core)
    core.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
