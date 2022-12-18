import rclpy
import torch
from rclpy.node import Node

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image  # , LaserScan
from std_msgs.msg import String, Float32MultiArray
import cv2
from yolov5.models import common
from yolov5.models.common import AutoShape
import os


class ImageProcessNode(Node):

    def __init__(self, useEdgeTPU, useConeDetection):
        super().__init__('image_process')
        self.declare_parameter('frequency', 2)
        self.frequency = self.get_parameter('frequency').value
        self.sub_internalCamStream = self.create_subscription(Image, 'internalCamStream',
                                                              self.callback, 10)
        self.sub_externalCommandStream = self.create_subscription(String, 'externalCommandStream',
                                                                  self.command_callback, 10)
        self.pub_externalCamStream = self.create_publisher(Image, 'externalCamStream', 10)
        self.pub_coneAssignment = self.create_publisher(Float32MultiArray, 'coneAssignment', 10)
        self.timer = self.create_timer(self.frequency, self.publish_callback)
        self.bridge = CvBridge()
        self.latest_frame = None

        self.model_filename = 'best-int8_edgetpu.tflite'
        self.PATH_TO_MODEL = "/home/ubuntu/allassignmens-32/src/camera_task/tflite_models/" + self.model_filename
        self.PATH_TO_LABELS = "/home/ubuntu/allassignmens-32/src/camera_task/tflite_models/labels.yaml"
        self.model = None
        if useConeDetection:
            if useEdgeTPU:
                self.setModelEdgeTPU()
            else:
                self.setModelCPU()
            self.model = AutoShape(self.model)
            self.model = self.model.to(torch.device('cpu'))
            self.model.conf = 0.25  # confidence threshold
            self.model.iou = 0.45  # NMS IoU threshold
            self.model.agnostic = False  # NMS class-agnostic
            self.model.multi_label = False  # NMS multiple labels per element

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
                if new_frequency < 0:
                    raise ValueError
                self.timer.cancel()
                self.frequency = new_frequency
                self.timer = self.create_timer(self.frequency, self.publish_callback)
                self.get_logger().info("Changed frequency to " + msg.data)
            except (TypeError, ValueError):
                self.get_logger().info("Received invalid input")

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
            # modify image
            if self.model is not None:
                cv_image = cv2.resize(cv_image, (640, 640))
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
                result = self.model(cv_image, augment=True)
                self.assignCones(result)
                result.render()
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            # mark image as ready to send
            self.setLatestFrame(cv_image)
        except Exception as err:
            self.get_logger().info(str(err))

    def assignCones(self, prediction):
        boundingBoxes = prediction.xyxy
        detections = Float32MultiArray()
        for tensor in boundingBoxes:
            for cone in tensor:
                detections.data.append(cone[0])  # left bottom x-coordinate of bBox
                detections.data.append(cone[2])  # right upper x-coordinate of bBox
                detections.data.append(cone[5])  # color (0=blue, 1=orange, 2=yellow)
        self.pub_coneAssignment.publish(detections)

    def getFrequency(self):
        return self.frequency

    def setFrequency(self, value):
        self.frequency = value

    def getLatestFrame(self):
        return self.latest_frame

    def setLatestFrame(self, image):
        self.latest_frame = image

    def setModelCPU(self):
        self.get_logger().info("Did not find Edge TPU, using CPU")
        self.model = common.DetectMultiBackend(
            weights="/home/ubuntu/allassignmens-32/src/camera_task/tflite_models/best-int8.tflite",
            data=self.PATH_TO_LABELS)

    def setModelEdgeTPU(self):
        try:
            self.model = common.DetectMultiBackend(
                weights=self.PATH_TO_MODEL,
                data=self.PATH_TO_LABELS)
        except ValueError:
            self.setModelCPU()


def main(args=None):
    rclpy.init(args=args)

    node = ImageProcessNode(True, True)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
