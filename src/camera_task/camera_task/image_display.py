# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
        Path("images/" + self.startTime).mkdir(parents=True, exist_ok=True)
        self.declare_parameter('window_width', 800)
        self.declare_parameter('window_height', 600)
        self.window_width = self.get_parameter('window_width').value
        self.window_height = self.get_parameter('window_height').value

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
            # self.get_logger().info("Received frame")
            resized_cv_image = cv2.resize(cv_image, (self.window_width, self.window_height))
            cv2.imshow("Stream", resized_cv_image)
            ret = cv2.waitKey(10)
            if ret == 27:  # esc key
                cv2.destroyAllWindows()
                sys.exit(0)
            cv2.imwrite("images/" + self.startTime + "/" + "frame" + str(self.ctr) + ".jpg",
                        cv_image)
            self.ctr += 1
        except CvBridgeError as err:
            self.get_logger().info(str(err))

    def handle_keyboard(self):
        while True:
            value = input(
                "Enter command \r\n<float-value>: Change frequency\r\nf: request picture)\r\n")
            msg = String()
            msg.data = str(value)
            self.pub_externalCommandStream.publish(msg)


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
