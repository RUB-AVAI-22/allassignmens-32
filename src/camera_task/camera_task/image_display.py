import sys

import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import threading
from datetime import datetime
from pathlib import Path
import os


class DisplayNode(Node):

    def __init__(self):
        super().__init__('image_display')
        self.sub_externalCamStream = self.create_subscription(Image, 'externalCamStream',
                                                              self.callback, 10)
        self.pub_externalCommandStream = self.create_publisher(String, 'externalCommandStream', 10)
        self.ctr = 0
        cv2.namedWindow("Stream", cv2.WINDOW_NORMAL)
        self.bridge = CvBridge()
        self.startTime = str(datetime.fromtimestamp(datetime.timestamp(datetime.now())))
        Path(os.path.dirname(__file__) + "/images/" + self.startTime).mkdir(exist_ok=True)
        self.declare_parameter('window_width', 800)
        self.declare_parameter('window_height', 600)
        self.window_width = self.get_parameter('window_width').value
        self.window_height = self.get_parameter('window_height').value
        self.enable_controls = True
        self.last_received_frame = None

    def callback(self, data):
        try:
            self.last_received_frame = self.bridge.imgmsg_to_cv2(data)
            # self.get_logger().info("Received frame")
            resized_cv_image = cv2.resize(self.last_received_frame, (self.window_width, self.window_height))
            cv2.imshow("Stream", resized_cv_image)
            ret = cv2.waitKey(10)
            if ret == 27:  # esc key
                cv2.destroyAllWindows()
                sys.exit(0)
            cv2.imwrite("images/" + self.startTime + "/" + "frame" + str(self.ctr) + ".jpg",
                        self.last_received_frame)
            self.ctr += 1
        except CvBridgeError as err:
            self.get_logger().info(str(err))

    def handle_keyboard(self):
        while self.enable_controls:
            value = input(
                "Enter command \r\n<float-value>: Change frequency\r\nf: request picture\r\nx: exit control)\r\n")
            self.checkExitControls(value)
            msg = String()
            msg.data = str(value)
            self.pub_externalCommandStream.publish(msg)

    def checkExitControls(self, value):
        if str(value) == 'x':
            self.enable_controls = False
            return

    def getLastReceivedFrame(self):
        return self.last_received_frame


def main(args=None):
    rclpy.init(args=args)

    node = DisplayNode()
    th = threading.Thread(target=node.handle_keyboard)
    th.start()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
