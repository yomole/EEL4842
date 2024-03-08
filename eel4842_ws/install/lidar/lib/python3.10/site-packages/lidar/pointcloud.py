import rclpy
import math

from rclpy.node import Node
from rclpy.clock import Clock

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import LaserScan

class MapPointCloud(Node):
    '''
    Converts joystick values to ackermann heading to be sent to the vehicle for its speed and steering angle.
    '''
    def __init__(self):
        super().__init__('pointcloud')       
        self.subLidar = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.publisher_ = self.create_publisher(PointCloud, 'pointcloud', 10)

    def lidar_callback(self,msg):
        msg_send = PointCloud()
        msg_send.header.frame_id = 'laser'

        num_points = len(msg.ranges)
        xy_points = [Point32() for i in range(num_points)]
        angle_inc = msg.angle_increment

        for i,r in enumerate(msg.ranges):
            calc_point = Point32()

            calc_point.x = r * math.cos((angle_inc * i) - math.pi)
            calc_point.y = r * math.sin((angle_inc * i) - math.pi)
            calc_point.z = float(0)

            xy_points[i] = calc_point

        msg_send.points = xy_points

        #Publish the message with steering and (maybe) speed.
        self.publisher_.publish(msg_send)

def main(args=None):
    rclpy.init(args=args)

    cloud_pub = MapPointCloud()
    
    rclpy.spin(cloud_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ack_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
