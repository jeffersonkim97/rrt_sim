import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
import numpy as np


# ROSBRIDGE COMMAND TO TRANSFER GZ POSE TO ROS2 TOPIC POSE
# ros2 run ros_gz_bridge parameter_bridge /model/target/pose@geometry_msgs/msg/Pose@gz.msgs.Pose

# ROSBRIDGE COMMAND TO TRANSFER ROS2 TOPIC TWIST TO GZ TWIST
# ros2 run ros_gz_bridge parameter_bridge /target/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
class Controller(Node):
    def __init__(self):
        super().__init__('Robot_Controller')
        # Subscriber for robot position
        self.target_pos_subscriber = self.create_subscription(Pose, "/model/target/pose", self.target_pose_callback, 10)

        # Publisher for robot control
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        #### ROBOT CONTROL PARAMETER ####
        # Forward velocity
        self.speed = 1

        # Current pos and orientation at given reference frame
        self.currentX = 0
        self.currentY = 0
        self.currentYaw = 0

        self.start_time = time.time()
        self.get_logger().info(str(self.start_time))

        self.waypoint = [-10, -10, self.start_time+5]

    def quaternion_to_euler_angle_vectorized(self, w, x, y, z):
        # convert Quaternion to euler angle
        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = np.degrees(np.arctan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = np.where(t2>+1.0,+1.0,t2)
        #t2 = +1.0 if t2 > +1.0 else t2

        t2 = np.where(t2<-1.0, -1.0, t2)
        #t2 = -1.0 if t2 < -1.0 else t2
        Y = np.degrees(np.arcsin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = np.degrees(np.arctan2(t3, t4))
        return X, Y, Z 

    def target_pose_callback(self, pose: Pose):
        # Target Position from Callback
        tx = pose.position.x
        ty = pose.position.y
        tz = pose.position.z
        
        # Target Orientation from Callback
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w

        # Extract euler angles
        pitch, roll, yaw = self.quaternion_to_euler_angle_vectorized(qw, qx, qy, qz)

        self.currentPose = [tx, ty, yaw]
        self.currentX = tx
        self.currentY = ty
        self.currentYaw = yaw
        # self.get_logger().info(str([self.currentX, self.currentY, self.currentYaw]))
    
        self.moveToPoint()

    def moveToPoint(self):
        # Move toward the waypoint
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        # Find Difference to Waypoint
        dx = self.waypoint[0]-self.currentX
        dy = self.waypoint[1]-self.currentX
        yawToWaypoint = np.arctan2(dy, dx)
        dt = self.waypoint[2] - self.start_time
        # print(dx, dy, dt, yawToWaypoint)

        eucleadianDist = np.sqrt(dx**2 + dy**2)
        
        # If vehicle travel in time:
        if eucleadianDist <= self.speed*dt:
            vehicleSpeed = eucleadianDist/dt
        else:
            vehicleSpeed = self.speed
        # print(vehicleSpeed)

        msg.linear.x = vehicleSpeed*np.cos(yawToWaypoint)
        msg.linear.y = vehicleSpeed*np.sin(yawToWaypoint)
        # print(vehicleSpeed*np.cos(yawToWaypoint), vehicleSpeed*np.sin(yawToWaypoint))

        self.publisher_.publish(msg=msg)

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    rclpy.shutdonw()

if __name__ == '__main__':
    main()