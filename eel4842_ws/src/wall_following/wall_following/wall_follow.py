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
        self.subJoy = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.subLidar = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'vehicle_command_ackermann', 10)

        self.declare_parameter('kd', 0.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('history', 45)
        self.declare_parameter('beam_angle', 10)

        self.prev_errors = [0 for i in range(self.get_parameter('history').value)]
        self.curr_error_index = 0
        self.ek_prev = 0
        self.prev_uk = None
        self.diff_uk = None
        self.uk = None
        self.autonomous = False

    def lidar_callback(self,msg):
        #Determine some fixed constants from the message.
        theta = msg.angle_increment
        smallest_range = msg.range_min
        largest_range = msg.range_max

        #Get the PID gain parameter values.
        kd = self.get_parameter('kd').value
        ki = self.get_parameter('ki').value
        kp = self.get_parameter('kp').value
        beam_angle = self.get_parameter('beam_angle').value
        
        #Get the PID storage history
        history = self.get_parameter('history').value

        print(f'Running node with kp = {kp}, ki = {ki}, kd = {kd}, history = {history}, beam_angle = {beam_angle}.')
        
        #Calculate the index of the first beam (90 deg from initial beam = to the right)
        d1_index = int(round((math.pi/2)/(theta),0))

        #Use the index of the next immediate beam to use the angle increment as the angle.
        d2_index = d1_index + beam_angle

        #Get the distances of these beams (if the data is valid).
        d1 = msg.ranges[d1_index] if (msg.ranges[d1_index] >= smallest_range and msg.ranges[d1_index] <= largest_range) else None
        d2 = msg.ranges[d2_index] if (msg.ranges[d2_index] >= smallest_range and msg.ranges[d1_index] <= largest_range) else None

        #Print a message if the data is invalid.
        if d1 == None or d2 == None:
            print('Wall is too close/far!')
            return
        
        #Otherwise, calculate the horizontal distance between the two beams (d3) and then use that for dwall.
        d3 = math.sqrt(d1**2 + d2**2 - (2 * d1 * d2 * math.cos(theta * beam_angle)))

        print(f'd1: {d1}\nd2: {d2}\nd3: {d3}\ntheta: {theta}\n')

        phi = math.asin((d1/d3)*math.sin(theta * beam_angle))
        dw = d1 * math.cos(90 - theta - phi)
        e = dw - 1

        print(f'phi: {phi}\ndwall: {dw}\nerror: {e}\n')

        if math.isnan(phi) or math.isnan(dw) or math.isnan(e):
            print('NAN Values detected! Scan will be discarded')
            return

        #Figure out variables associated with PID.
        dt = msg.scan_time
        sum_ek = sum(self.prev_errors)
        print(f'current error: {self.curr_error_index}, history: {history}')
        self.prev_errors[self.curr_error_index % history] = e
        self.curr_error_index += 1

        #Calculate the control input.
        self.uk = (kp * e) + (ki * sum_ek * dt) + (kd * ((e - self.ek_prev)/dt))
        self.ek_prev = e

        #Calculate difference in control input if valid.

        if (self.prev_uk != None):
            self.diff_uk = self.uk - self.prev_uk

        self.prev_uk = self.uk

        print(f'uk: {self.uk}, delta_uk: {self.diff_uk}')
        print(f'previous errors:{self.ek_prev}\n')
        print(f'Steering angle: {(math.pi/4) * self.uk} radians ({((math.pi/4) *self.uk)*(180/math.pi)} degrees)')

        #limits for steering angle are -45 to 45
        #limits for speed is 70 to 100
        #Anything between -0.05 and 0.05 is 0 (to keep the car stationary).

        if (not self.autonomous):
            return

        msg_send = AckermannDriveStamped()
        msg_send.drive.steering_angle = -(math.pi/4) * self.uk   
    
        print(f'Steering angle: {msg_send.drive.steering_angle} radians ({msg_send.drive.steering_angle * (180 / math.pi)} degrees)')

        speed = 10
    
        #Only calculate and send speed if it is above a cutoff (defined as 0.05).
        msg_send.drive.speed = float(speed)
        print(f'Speed: {msg_send.drive.speed}')

        #Publish the message with steering and (maybe) speed.
        self.publisher_.publish(msg_send)

    def joy_callback(self, msg):
        if (msg.buttons[0] == 1):
            self.autonomous = True
            return
        else:
            self.autonomous = False

        msg_send = AckermannDriveStamped()
        msg_send.drive.steering_angle = float((msg.axes[0] * math.pi/4))

        #print(f'Steering angle: {msg_send.drive.steering_angle} radians ({msg_send.drive.steering_angle * (180 / math.pi)} degrees)')
        speed = abs(msg.axes[4])
        
        #Only calculate and send speed if it is above a cutoff (defined as 0.05).
        if speed >= 0.05:

            velocity_sign = 1
            if msg.axes[4] < 0:
                velocity_sign = -1

            msg_send.drive.speed = velocity_sign * float(((30 * speed)/0.95)+70)

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
