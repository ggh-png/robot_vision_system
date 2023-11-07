import rclpy
from rclpy.node import Node
from robotvisionsystem_msgs.msg import State

class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')
        self.sub_state = self.create_subscription(State, '/car/state', self._state, 10)

        # 현재의 x와 y 좌표를 저장
        self.current_x = 0.0
        self.current_y = 0.0

    def _state(self, data):
        self.current_x = data.pos_x
        self.current_y = data.pos_y

        # 받은 데이터를 path.txt에 저장
        with open('path.txt', 'a') as f:
            f.write(f"{self.current_x},{self.current_y}\n")

def main(args=None):
    rclpy.init(args=args)

    path_recorder = PathRecorder()

    rclpy.spin(path_recorder)

    path_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
