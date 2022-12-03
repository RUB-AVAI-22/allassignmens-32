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

import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


# from random import randrange

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
        success, frame = self.video.read()
        try:
            if success:
                # frame = cv2.imread('/home/ubuntu/share/cones_labeled_1/frame' + str(randrange(
                # 0,115)) + '.jpg') Debugging for detection
                self.publisher.publish(self.bridge.cv2_to_imgmsg(frame))
                self.get_logger().info("Publishing video frame")
        except CvBridgeError as err:
            self.get_logger().info(str(err))


def main(args=None):
    rclpy.init(args=args)
    pub = CameraNode()
    rclpy.spin(pub)

    pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
