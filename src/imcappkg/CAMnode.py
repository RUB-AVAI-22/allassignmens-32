import sys
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from colorama import Fore
from colorama import Style

class CameraNode(Node):
    def __init__(self):
        super().__init__('Camnode')
        self.declare_parameter('fps',1.0) # This parameter sets the frame per second frecuency of image captureing
        self.fps= self.get_parameter('fps')
        self.count = 0
        self.publisher_ = self.create_publisher(Image, 'CAM', 10)
        self.timer = self.create_timer(self.fps.value, self.publish_callback)
        self.add_on_set_parameters_callback(self.fps_callback)
        self.br = CvBridge()
        self.cap = cv2.VideoCapture(0)

    def publish_callback(self):
        try:
            ret, frame = self.cap.read()
            if ret == True:
                #cv2.imshow("camera",frame)
                self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
                self.get_logger().info(f'{Fore.YELLOW}Publishing video frame_{self.count}to Topic CAM{Style.RESET_ALL}')
            self.count = self.count + 1
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.cap.release()
                cv2.destroyAllWindows()
                sys.exit()
        except Exception as e:
            print(e)

    def fps_callback(self,params):
        for param in params:
            if param.name == 'fps' and param.type_ == Parameter.Type.DOUBLE:
                self.fps.value = param.value
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    cam_node = CameraNode()
    rclpy.spin(cam_node)
    cam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()