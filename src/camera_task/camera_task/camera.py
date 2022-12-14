import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class CameraNode(Node):

    def __init__(self):
        super().__init__('cam')
        self.publisher = self.create_publisher(Image, 'internalCamStream', 10)
        self.timer = self.create_timer(0, self.video_callback)
        self.video = cv2.VideoCapture(0)  # 0, 14, "rtsp://web.nidaku.de:8554/avai"
        self.video.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.video.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.bridge = CvBridge()

    def video_callback(self):
        success, frame = self.getVideoFrame()
        try:
            if success:
                self.publisher.publish(self.bridge.cv2_to_imgmsg(frame))
                self.get_logger().info("Publishing video frame")
        except CvBridgeError as err:
            self.get_logger().info(str(err))

    def getVideoFrame(self):
        return self.video.read()


def main(args=None):
    rclpy.init(args=args)
    pub = CameraNode()
    rclpy.spin(pub)

    pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
