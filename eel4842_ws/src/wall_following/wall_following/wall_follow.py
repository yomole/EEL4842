''' Author:         Dewayne Roxborough
    Date:           March 1, 2024 
    Subject:        EML4842 | HW6 - PID wall following
    Description:    Dive along wall while maintaining 1 meter distance from the wall

'''
import random as rnd;
import rclpy;
import math;

from rclpy.node import Node;
from sensor_msgs.msg import Joy;
from sensor_msgs.msg import LaserScan;
from ackermann_msgs.msg import AckermannDriveStamped;
from std_msgs.msg import Float32;
from std_msgs.msg import Int32

class WallFollowNode(Node):

    def __init__(self):

        super().__init__("wall_follow");

    #   Define Initial Variables
    #   ************************    
        self.theta_fixed = 45;      # Angle used for d2
        self.theta90_index = -1;    # 90 degree index of lidar for d1, updated after each message received
        self.wall_distance = 1.0;   # Distance to maintain from wall in meters
        self.current_msg = None;    # Will store current message from lidar scan
        self.dt   = 0;
        self.declare_parameter('Kp', 0.8);
        self.declare_parameter('Kd', 1.0);
        self.declare_parameter('Ki', 2.0);
        self.declare_parameter('Speed', 70);
        self.declare_parameter('Angle_Limit', (math.pi/4))
        self.e_k  = 0;
        self.e_k1 = 0;
        self.e_k2 = 0;
        
        self.Is_dValsValid = False;

        #self.Kp = 0.8
        #self.Kd = 0
        #self.Ki = 0
        #self.Speed = 70
        #self.Limit = math.pi/4

        self.Kp = self.get_parameter('Kp').value
        self.Kd = self.get_parameter('Kd').value
        self.Ki = self.get_parameter('Ki').value
        self.Speed = self.get_parameter('Speed').value
        self.Limit = self.get_parameter('Angle_Limit').value

        #self.subscription1  = self.create_subscription(msg_type = Joy, topic = "joy", callback = self.data_received1, qos_profile = 1);
        self.subscription2  = self.create_subscription(msg_type = LaserScan, topic = "scan", callback = self.do_follow_wall, qos_profile = 1);
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'vehicle_command_ackermann', 10)

#   INITIAL CALLBACK FUNCTION FROM LIDAR SCAN - IT ALSO UPDATES CLASS PROPERTIES:
#   *****************************************************************************
    def do_follow_wall(self, msg = LaserScan):
        self.current_msg = msg;
        dwall = self.get_dWall();
        err = dwall - self.wall_distance;    # Also detects invalid d1 or d2 values

        #self.get_logger().info(f'self.Is_dValsValid = {self.Is_dValsValid}');
        if (self.Is_dValsValid):

            self.dt   = msg.time_increment;
            self.e_k2 = self.e_k1;
            self.e_k1 = self.e_k;
            self.e_k  = err;
        
        #   Get du or u (angle to sent to car)
        #   **********************************
            u = self.get_u();
            du = self.get_du();

            # TO DO... CODE: use either of the values above to steer the car

            msg_send = AckermannDriveStamped()

            msg_send.drive.speed = float(self.Speed)
            msg_send.drive.steering_angle = float(u)
            self.publisher_.publish(msg_send)

            self.get_logger().info(f'err = {err}, dt = {self.dt}, e_k = {self.e_k}, e_k1 = {self.e_k1}, e_k2 = {self.e_k2}');
            self.get_logger().info(f'u = {u}, du = {du}, dWall = {dwall}');

        #   Possible block format to steer car
        #   **********************************
            if (err > 0):
                i = 0; # Turn right
            elif (err < 0):
                i = 1; # Turn left
        else:
            self.get_logger().info(f'Either d1 or d2 is invalid.  Commands ignored.');
        

    def get_u(self):
        kp   = self.Kp;
        kd   = self.Kd;
        ki   = self.Ki;
        dt   = self.dt;
        e_k  = self.e_k;
        e_k1 = self.e_k1;
        u    = kp*e_k + ki*e_k*dt + kd*(e_k - e_k1)/dt;

        if (u < -self.Limit):
            u = -self.Limit;
        elif (u > self.Limit):
            u = self.Limit;
        
        return u;


    def get_du(self):
        kp   = self.Kp;
        kd   = self.Kd;
        ki   = self.Ki;
        dt   = self.dt;
        e_k  = self.e_k;
        e_k1 = self.e_k1;
        e_k2 = self.e_k2;
        du   = kp*(e_k - e_k1) + ki*e_k*dt + kd*(e_k - 2*e_k1 + e_k2)/dt;

        if (du < -self.Limit):
            du = -self.Limit;
        elif (du > self.Limit):
            du = self.Limit;
        
        return du;


    def get_dWall(self):
        min     = self.current_msg.angle_min;
        inc     = self.current_msg.angle_increment;
        rngs    = self.current_msg.ranges;
        th_d2_i = self.get_theta_i(self.current_msg, self.theta_fixed);     # Get the index for the d2 angle
        self.theta90_index = self.get_theta_i(self.current_msg, 90);        # Get the index for the d1 angle @ 90 degrees
        d1      = self.get_d_vals(rngs, min, inc, self.theta90_index);
        d2      = self.get_d_vals(rngs, min, inc, th_d2_i);

        self.get_logger().info(f'd1 = {d1[0]}, d2 = {d2[0]}, th_d1_i = {self.theta90_index}, th_d2_i = {th_d2_i}');

        if not(abs(d1[0]) > 0 and abs(d2[0]) > 0):                          # Update valid flag and return default (null) value if not valid
            self.Is_dValsValid = False;
            return None;
        else:
            self.Is_dValsValid = True;

        d3      = self.get_d3(d1[0], d2[0], self.theta_fixed);
        cos_phi = (d2[1]-d1[1])/d3;

        self.get_logger().info(f'd3 = {d3}, cos_phi = {cos_phi}');

        dwall   = d1[0]*cos_phi;
        return  dwall;


    def get_theta_i(self, msg = LaserScan, theta = Float32):
        rngs = msg.ranges;
        for i in range(len(rngs)):
            th = (msg.angle_min + (i)*msg.angle_increment)*180/math.pi;
            if ((th < (theta + 0.3)) and (th >= theta)):
                return i;
        return 0;                                                           # Value return if nothing found
    

    def get_d3(self, d1 = Float32, d2 = Float32, theta = Float32):
        d3 = math.sqrt(d1**2 + d2**2 - 2*d1*d2*math.cos(theta));
        return d3;

#   Return the d1 or d2 distance from wall based on the index of a theta value:
#   ***************************************************************************
    def get_d_vals(self, rngs = [], angle_min = Float32, angle_inc = Float32, theta_i = Int32):
        th = (angle_min + theta_i*angle_inc)*180/math.pi;
        x = rngs[theta_i]*math.cos(th);
        y = rngs[theta_i]*math.sin(th);
        d = rngs[theta_i];
        return [d, x, y];

def main(args=None):

    rclpy.init(args=args);
    node_template = WallFollowNode();
    rclpy.spin(node_template);
    node_template.destroy_node();
    rclpy.shutdown();

if __name__ == "__main__":
    main();
