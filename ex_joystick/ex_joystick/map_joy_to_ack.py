import rclpy
import math

from rclpy.node import Node
from rclpy.clock import Clock

from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy

class JoyToAckNode(Node):
    '''
    Converts joystick values to ackermann heading.
    '''
    def __init__(self):
        super().__init__('joy')       
        self.subHead = self.create_subscription(Joy, 'joy', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'vehicle_command_ackermann', 10)

    def listener_callback(self,msg):
        print(f'joy message test:\naxes:\n{msg.axes}')


        msg = AckermannDriveStamped()
        msg.drive.speed = float(10)
        msg.drive.steering_angle = float(math.pi/6)

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    ack_pub = JoyToAckNode()
    
    rclpy.spin(ack_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ack_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
