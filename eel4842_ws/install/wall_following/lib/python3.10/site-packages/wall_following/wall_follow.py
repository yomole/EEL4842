import rclpy
import math

from rclpy.node import Node
from rclpy.clock import Clock

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan

class JoyToAckNode(Node):
    '''
    Converts joystick values to ackermann heading to be sent to the vehicle for its speed and steering angle.
    '''
    def __init__(self):
        super().__init__('map_joy_to_ack')       
        self.subJoy = self.create_subscription(Joy, 'joy', self.listener_callback, 10)
        self.subLidar = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'vehicle_command_ackermann', 10)

    def lidar_callback(self,msg):
        theta = msg.angle_increment
        smallest_range = msg.range_min
        largest_range = msg.range_max

        d1_index = int(round((math.pi/2)/(theta),0))
        d2_index = d1_index+1

        d1 = msg.ranges[d1_index] if (msg.ranges[d1_index] >= smallest_range and msg.ranges[d1_index] <= largest_range) else None
        d2 = msg.ranges[d2_index] if (msg.ranges[d2_index] >= smallest_range and msg.ranges[d1_index] <= largest_range) else None

        if d1 == None or d2 == None:
            print('Wall is too close/far!')
        
        else:
            d3 = math.sqrt(d1**2 + d2**2 - (2 * d1 * d2 * math.cos(theta)))

            print(f'd1: {d1}\nd2: {d2}\nd3: {d3}\ntheta: {theta}\n')

            phi = math.asin((d1/d3)*math.sin(theta))
            dw = d1 * math.cos(90 - theta - phi)
            e = dw - 1

            print(f'phi: {phi}\ndwall: {dw}\nerror: {e}\n')

    def listener_callback(self,msg):
        #limits for steering angle are -45 to 45
        #limits for speed is 70 to 100
        #Anything between -0.05 and 0.05 is 0 (to keep the car stationary).

        msg_send = AckermannDriveStamped()
        msg_send.drive.steering_angle = 0   
        
        print(f'Steering angle: {msg_send.drive.steering_angle} radians ({msg_send.drive.steering_angle * (180 / math.pi)} degrees)')

        speed = 0
        
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
