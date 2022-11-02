import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ImageProcessNode(Node):

    def __init__(self):
        super().__init__('image_process')
        self.declare_parameter('frequency', 5)
        self.frequency = self.get_parameter('frequency').value
        self.sub_internalCamStream = self.create_subscription(Image, 'internalCamStream', self.callback, 10)
        self.sub_externalCommandStream = self.create_subscription(String, 'externalCommandStream', self.command_callback, 10)
        self.pub_externalCamStream = self.create_publisher(Image, 'externalCamStream', 10)
        self.timer = self.create_timer(self.frequency, self.publish_callback)
        self.bridge = CvBridge()
        self.latest_frame = None

    def publish_callback(self):
        try:
            if self.latest_frame is not None:
                self.pub_externalCamStream.publish(self.bridge.cv2_to_imgmsg(self.latest_frame))
                self.get_logger().info("Publishing video frame")
        except CvBridgeError as error:
            self.get_logger().info(str(error))

    def command_callback(self, msg):
        self.get_logger().info("Received " + msg.data + " as command")
        if msg.data == 'f':
            self.publish_callback()
        else:
            try:
                new_frequency = float(msg.data)
                self.timer.cancel()
                self.frequency = new_frequency
                self.timer = self.create_timer(self.frequency, self.publish_callback)
                self.get_logger().info("Changed frequency to " + msg.data)
            except Exception:
                self.get_logger().info("Received invalid input")

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
            # modify image
            self.latest_frame = cv_image
        except CvBridgeError as err:
            self.get_logger().info(str(err))


def main(args=None):
    rclpy.init(args=args)

    node = ImageProcessNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
