import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class LineTrackerOdomNode(Node):
    def __init__(self):
        super().__init__('line_tracker_odom_node')
        
        self.left_cmd_pub = self.create_publisher(Float32, '/left_wheel_cmd', 10)
        self.right_cmd_pub = self.create_publisher(Float32, '/right_wheel_cmd', 10)
        self.path_pub = self.create_publisher(Path, '/robot_line_path', 10)

        self.sensor_sub = self.create_subscription(Int32MultiArray, '/sensor_readings', self.sensor_callback, 10)
        self.left_enc_sub = self.create_subscription(Float32, '/left_wheel_state', self.left_enc_callback, 10)
        self.right_enc_sub = self.create_subscription(Float32, '/right_wheel_state', self.right_enc_callback, 10)

        # Vehicle Kinematics Parameters
        self.WHEEL_DIAMETER = 0.065 
        self.WHEEL_BASE = 0.15      
        self.TPR = 360.0            

        # Global Odometry State Variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_l_ticks = 0.0
        self.last_r_ticks = 0.0
        self.curr_l_ticks = 0.0
        self.curr_r_ticks = 0.0
        self.first_run = True

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

    def left_enc_callback(self, msg):
        self.curr_l_ticks = msg.data
        self.calculate_odometry()

    def right_enc_callback(self, msg):
        self.curr_r_ticks = msg.data
        self.calculate_odometry()

    def calculate_odometry(self):
        if self.first_run:
            self.last_l_ticks = self.curr_l_ticks
            self.last_r_ticks = self.curr_r_ticks
            self.first_run = False
            return

        d_left = self.curr_l_ticks - self.last_l_ticks
        d_right = self.curr_r_ticks - self.last_r_ticks

        dist_left = (d_left / self.TPR) * math.pi * self.WHEEL_DIAMETER
        dist_right = (d_right / self.TPR) * math.pi * self.WHEEL_DIAMETER
        delta_s = (dist_left + dist_right) / 2.0
        delta_theta = (dist_right - dist_left) / self.WHEEL_BASE

        self.x += delta_s * math.cos(self.theta + delta_theta / 2.0)
        self.y += delta_s * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

        self.last_l_ticks = self.curr_l_ticks
        self.last_r_ticks = self.curr_r_ticks

        self.publish_path()

    def publish_path(self):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        
        self.path_msg.poses.append(pose)
        self.path_msg.header.stamp = pose.header.stamp
        self.path_pub.publish(self.path_msg)

    def sensor_callback(self, msg):
        s = msg.data
        
        # Calibrated Velocity Profile
        cruise = 135.0
        steer = 175.0
        pivot = 75.0
        sharp_s = 170.0
        sharp_r = -110.0

        left_speed = 0.0
        right_speed = 0.0

        if s[2] == 1:   
            left_speed, right_speed = cruise, cruise
        elif s[1] == 1: 
            left_speed, right_speed = pivot, steer
        elif s[3] == 1: 
            left_speed, right_speed = steer, pivot
        elif s[0] == 1: 
            left_speed, right_speed = sharp_r, sharp_s
        elif s[4] == 1: 
            left_speed, right_speed = sharp_s, sharp_r
        else:
            left_speed, right_speed = 0.0, 0.0

        l_out = Float32()
        r_out = Float32()
        l_out.data = left_speed
        r_out.data = right_speed
        
        self.left_cmd_pub.publish(l_out)
        self.right_cmd_pub.publish(r_out)

def main(args=None):
    rclpy.init(args=args)
    node = LineTrackerOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
