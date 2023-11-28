import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from robotvisionsystem.Logger import Logger


from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from robotvisionsystem_msgs.msg import State
from robotvisionsystem_msgs.msg import Ray


from robotvisionsystem.Logger import Logger
from robotvisionsystem.LaneDetector import LaneDetector
from robotvisionsystem.TrafficLightDetector import TrafficLightDetector
from robotvisionsystem.StopLineDetector import StopLineDetector


class Sensor:
    def __init__(self, node: Node):
        # Ensure that the node argument is indeed an instance of rclpy.node.Node
        if not isinstance(node, Node):
            raise TypeError("Logger expects an rclpy.node.Node instance.")
        self.node = node
        self.logger = Logger(self.node)
        self.cv_bridge = CvBridge()

        # detector
        # 차선 중심점
        self.lane_detector = LaneDetector(self.node)
        # stop line
        self.stopline_detector = StopLineDetector(self.node)
        # traffic light
        self.traffic_light_detector = TrafficLightDetector(self.node)

        # Odometry
        self.sub_state = self.node.create_subscription(
            State, '/car/state', self.state_callback, 10)

        self.sub_camera = self.node.create_subscription(
            Image, '/car/sensor/camera/front', self.camera_callback, 10)

        self.sub_ray = self.node.create_subscription(
            Ray, '/car/sensor/ray', self.ray_callback, 10)

        # Sensor data
        self.state_msg = None
        self.camera_msg = None
        self.ray_msg = None

        self.centor_lane = None
        self.stopline = None
        self.traffic_light = None

    def state_callback(self, msg):
        # self.node.get_logger().info('Odometry callback triggered')
        self.odom_msg = msg

    def camera_callback(self, msg):
        # Convert the ROS Image message to OpenCV format
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
            self.camera_msg = cv_image  # Assign the converted image to camera_msg
        except CvBridgeError as e:
            self.logger.error('Failed to convert image: %s' % str(e))

        self.centor_lane = self.lane_detector(self.camera_msg)
        # self.logger.info(self.centor_lane)
        self.stopline = self.stopline_detector(self.camera_msg)
        self.traffic_light = self.traffic_light_detector(self.camera_msg)

    def ray_callback(self, msg):
        self.ray_msg = msg.ray_array

    def init(self):
        # self.logger.info("wcrc_ctrl sensor wait...")
        # if self.centor_lane is not None and self.stopline is not None and self.traffic_light is not None:
        if self.centor_lane is not None:
            return True
        else:
            return False
