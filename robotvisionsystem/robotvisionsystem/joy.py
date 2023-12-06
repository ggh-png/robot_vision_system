import rclpy
from rclpy.node import Node
from robotvisionsystem_msgs.msg import Motor
import pygame
from pygame.locals import *


class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher_ = self.create_publisher(Motor, '/car/motor', 10)
        self.motor_msg = Motor()
        pygame.init()
        screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption('Motor Controller')
        self.run_controller()

    def run_controller(self):
        running = True
        while running and rclpy.ok():
            for event in pygame.event.get():
                if event.type == QUIT:
                    running = False
                if event.type == KEYDOWN:
                    self.motor_msg.motorspeed = 0.22
                    self.publisher_.publish(self.motor_msg)
                    self.update(event.key)
                    # self.motor_msg.motorspeed = 0.08
                    self.publisher_.publish(self.motor_msg)

    def update(self, key):
        if key == K_LEFT:
            self.motor_msg.steer = -2.5
            self.motor_msg.steer -= 15.0
            self.motor_msg.motorspeed = 0.0
            self.motor_msg.breakbool = False
        elif key == K_RIGHT:
            self.motor_msg.steer = 2.5
            self.motor_msg.steer += 15.0
            self.motor_msg.motorspeed = 0.0
            self.motor_msg.breakbool = False
        elif key == K_UP:
            self.motor_msg.motorspeed += 0.02
            self.motor_msg.steer = 0.0
            self.motor_msg.breakbool = False
        elif key == K_DOWN:
            self.motor_msg.motorspeed = 0.0
            self.motor_msg.motorspeed -= 0.2
            self.motor_msg.steer = 0.0
        elif key == K_b:
            self.motor_msg.breakbool = not self.motor_msg.breakbool


def main(args=None):
    rclpy.init(args=args)
    keyboard_controller = KeyboardController()
    rclpy.spin(keyboard_controller)
    keyboard_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
