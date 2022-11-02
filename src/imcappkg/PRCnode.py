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

class ProcesNode(Node):
    def __init__(self):
        super().__init__('Procnode')
        self.declare_parameter('PRtoSTORAGE',1.0) # This parameter sets the frecuency at which the Node publishes to STORAGE
        self.prs = self.get_parameter('PRtoSTORAGE')
        self.count = 0
        self.count_s =0
        self.br = CvBridge()
        self.current_frame = None
        self.subscription = self.create_subscription(Image,'CAM',self.subscribe_callback,10)
        self.subscription
        self.publisher_ = self.create_publisher(Image, 'STORAGE', 10)
        self.timer = self.create_timer(self.prs.value, self.publish_callback)
        self.add_on_set_parameters_callback(self.PRtoSTORAGE_callback)

    def subscribe_callback(self, data):
        try:
            self.current_frame = self.br.imgmsg_to_cv2(data)
            self.get_logger().info(f'{Fore.CYAN}Receiving video frame_{self.count} from Topic CAM{Style.RESET_ALL}')
            self.count = self.count + 1
            #```
            # some processing like CNN would take place here !
            #cv2.imshow("Camera", self.current_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                sys.exit()
            #```
        except Exception as e:
            print(e)

    def publish_callback(self):
        try:
            self.publisher_.publish(self.br.cv2_to_imgmsg(self.current_frame,encoding="passthrough"))
            self.get_logger().info(f'{Fore.LIGHTBLUE_EX}Publishing video frame_{self.count_s} to Topic STORAGE{Style.RESET_ALL}')
            self.count_s = self.count_s + 1
        except Exception as e:
            print(e)

    def PRtoSTORAGE_callback(self,params):
        for param in params:
            if param.name == 'PRtoSTORAGE' and param.type_ == Parameter.Type.DOUBLE:
                self.prs.value = param.value + 0.1
        return SetParametersResult(successful=True)

def main():
    rclpy.init()
    image_subscriber = ProcesNode()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()