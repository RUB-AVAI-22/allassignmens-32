import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty, termios


class remote_control(Node):

    def __init__(self):
        super().__init__('remote_control')
        self.settings = termios.tcgetattr(sys.stdin)
        self.maxLinearVelocity = 0.26
        self.maxAngularVelocity = 1.82
        self.linearVelocityChangeValue = 0.01
        self.angularVelocityChangeValue = 0.1
        self.linearVelocity = 0.0 #backward/forward
        self.angularVelocity = 0.0 #sideways
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0, self.callback)

    def callback(self):
        key = self.getKey()
        twist = Twist()
        if key == 'w':
            self.linearVelocity += self.linearVelocityChangeValue
        if key == 's':
            self.linearVelocity -= self.linearVelocityChangeValue
        if key == 'a':
            self.angularVelocity += self.angularVelocityChangeValue
        if key == 'd':
            self.angularVelocity -= self.angularVelocityChangeValue
        if key == 'e':
            self.linearVelocity = 0.0
            self.angularVelocity = 0.0

        twist.linear.x = self.linearVelocity
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.angularVelocity
        self.get_logger().info("linearVelocity = " + str(self.linearVelocity) + " angularVelocity = " + str(self.angularVelocity))
        self.publisher_.publish(twist)

    def getKey(self):
        tty.setcbreak(sys.stdin)
        x = 0
        while x == 0:
            x = sys.stdin.read(1)[0]
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return x

def main(args=None):
    rclpy.init(args=args)
    pub = remote_control()
    rclpy.spin(pub)

    pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
