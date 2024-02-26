import rclpy
import math

from rclpy.node import Node
from rclpy.clock import Clock

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy

class JoyToAckNode(Node):
    '''
    Converts joystick values to ackermann heading to be sent to the vehicle for its speed and steering angle.
    '''
    def __init__(self):
        super().__init__('map_joy_to_ack')       
        self.subHead = self.create_subscription(Joy, 'joy', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'vehicle_command_ackermann', 10)

    def listener_callback(self,msg):
        #limits for steering angle are -45 to 45
        #limits for speed is 70 to 100
        #Anything between -0.05 and 0.05 is 0 (to keep the car stationary).

        #axes[4] is y
        #axes[0] is steering x (left is positive, right is negative)

        #Create the ackermann message and calculate steering.
        msg_send = AckermannDriveStamped()
        msg_send.drive.steering_angle = float((msg.axes[0] * math.pi/4))
    
        print(f'Steering angle: {msg_send.drive.steering_angle} radians ({msg_send.drive.steering_angle * (180 / math.pi)} degrees)')

        speed = abs(msg.axes[4])
        
        #Only calculate and send speed if it is above a cutoff (defined as 0.05).
        if speed >= 0.05:

            velocity_sign = 1
            if msg.axes[4] < 0:
                velocity_sign = -1

            msg_send.drive.speed = velocity_sign * float(((30 * speed)/0.95)+70)

            print(f'Speed: {msg_send.drive.speed}')

        #Publish the message with steering and (maybe) speed.
        self.publisher_.publish(msg_send)

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
