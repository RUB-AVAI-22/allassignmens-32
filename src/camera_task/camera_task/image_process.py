import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy
from pycoral.adapters import common
from pycoral.adapters import detect
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter


class ImageProcessNode(Node):

    def __init__(self):
        super().__init__('image_process')
        self.declare_parameter('frequency', 5)
        self.frequency = self.get_parameter('frequency').value
        self.sub_internalCamStream = self.create_subscription(Image, 'internalCamStream', self.callback, 10)
        self.sub_externalCommandStream = self.create_subscription(String, 'externalCommandStream',
                                                                  self.command_callback, 10)
        self.pub_externalCamStream = self.create_publisher(Image, 'externalCamStream', 10)
        self.timer = self.create_timer(self.frequency, self.publish_callback)
        self.bridge = CvBridge()
        self.latest_frame = None
        self.model_filename = 'best-int8_edgetpu.tflite'
        self.PATH_TO_MODEL = '/home/ubuntu/allassignmens-32/src/camera_task/tflite_models/' + self.model_filename
        self.labels = ["blue", "orange", "yellow"]
        self.interpreter = make_interpreter(self.PATH_TO_MODEL)
        self.interpreter.allocate_tensors()

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
            self.latest_frame = self.detect_and_draw_boxes(cv_image)
        except CvBridgeError as err:
            self.get_logger().info(str(err))

    def detect_and_draw_boxes(self, frame):
        _, scale = common.set_resized_input(self.interpreter, (640, 480), lambda size: cv2.resize(frame, (640, 480)))
        self.interpreter.invoke()
        objs = detect.get_objects(self.interpreter, 0.5, scale)

        for obj in objs:
            bbox = obj.bbox
            cv2.rectangle(frame, (bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax), (10, 255, 0), 2)
        return frame


def main(args=None):
    rclpy.init(args=args)

    node = ImageProcessNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
