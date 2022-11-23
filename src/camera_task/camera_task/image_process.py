import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tensorflow.lite.python.interpreter import load_delegate
import cv2
import numpy

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
        self.model_filename = 'best-int8_edgetpu.tflite'
        self.PATH_TO_MODEL = '/home/ubuntu/allassignmens-32/src/camera_task/tflite_models/' + self.model_filename
        self.labels = ["blue", "orange", "yellow"]
        self.interpreter = Interpreter(model_path=self.PATH_TO_MODEL, experimental_delegates=[load_delegate('libedgetpu.so.1.0')])
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.height = self.input_details[0]['shape'][1]
        self.width = self.input_details[0]['shape'][2]


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
            self.latest_frame = self.detect_and_draw_boxes(self, cv_image)
        except CvBridgeError as err:
            self.get_logger().info(str(err))

    def detect_and_draw_boxes(self, frame):
        frame_resized = cv2.resize(frame, (self.width, self.height))
        input_data = numpy.expand_dims(frame_resized, axis=0)
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()
        boxes = self.interpreter.get_tensor(self.output_details[0]['index'])[0]  # Bounding box coordinates of detected objects
        classes = self.interpreter.get_tensor(self.output_details[1]['index'])[0]  # Class index of detected objects
        scores = self.interpreter.get_tensor(self.output_details[2]['index'])[0]  # Confidence of detected objects
        for i in range(len(scores)):
            # Get bounding box coordinates and draw box
            # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
            ymin = int(max(1, (boxes[i][0] * 480)))
            xmin = int(max(1, (boxes[i][1] * 640)))
            ymax = int(min(480, (boxes[i][2] * 480)))
            xmax = int(min(640, (boxes[i][3] * 640)))

            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (10, 255, 0), 2)

            # Draw label
            object_name = self.labels[int(classes[i])]  # Look up object name from "labels" array using class index
            label = '%s: %d%%' % (object_name, int(scores[i] * 100))  # Example: 'person: 72%'
            labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)  # Get font size
            label_ymin = max(ymin, labelSize[1] + 10)  # Make sure not to draw label too close to top of window
            cv2.rectangle(frame, (xmin, label_ymin - labelSize[1] - 10),
                          (xmin + labelSize[0], label_ymin + baseLine - 10), (255, 255, 255),
                          cv2.FILLED)  # Draw white box to put label text in
            cv2.putText(frame, label, (xmin, label_ymin - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0),
                        2)  # Draw label text
        return frame


def main(args=None):
    rclpy.init(args=args)

    node = ImageProcessNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
