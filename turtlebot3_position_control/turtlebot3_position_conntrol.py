#!/usr/bin/env python3

import math
import numpy


from geometry_msgs.msg import Twist
from rclpy.node import Node
#from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry,Path

from turtlebot3_ppath import Turtlebot3Path




class Turtlebot3PositionControl(Node):

    def __init__(self):
        super().__init__('turtlebot3_position_control')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.odom = Odometry()
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.last_pose_theta = 0.0
        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0
        self.goal_pose_theta = 0.0
        self.step = 1
        self.init_odom_state = False  # To get the initial pose at the beginning
        self.init_path=False
        self.path=None

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        #qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialise subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        self.planned_path_sub = self.create_subscription(
            Path,  # Assuming OMPL publishes a path as PoseStamped messages
            '/planned_path',
            self.planned_path_callback,
            10)

        """************************************************************
        ** Initialise timers
        ************************************************************"""
        self.update_timer = self.create_timer(0.010, self.update_callback)  # unit: s

        self.get_logger().info("Turtlebot3 position control node has been initialised.")

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def odom_callback(self, msg):
        self.last_pose_x = msg.pose.pose.position.x
        self.last_pose_y = msg.pose.pose.position.y
        _, _, self.last_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation)

        self.init_odom_state = True
    def planned_path_callback(self,path_msg):
    	if path_msg.poses:
            self.get_logger().info("Recevied planned path message")
            self.path=path_msg   
            self.init_path=True
            		
    	
    def update_callback(self):
        if self.init_odom_state and self.init_path :
            self.generate_path()

    def generate_path(self):
        twist = Twist()
        lt=[[-1.9999,0],[-0.01224,-0.166647]]
        tf=numpy.array(lt)
        target=None
        for pose_stamped in self.path.poses:
            
            pose = pose_stamped.pose
            position = pose.position
            orientation = pose.orientation
            target=numpy.array([position.x,position.y])
            target=numpy.matmul(lt,target)
            dx = 0 - (-1.24)
            dy = 0 - (-2.4)
            _, _, theta = self.euler_from_quaternion(orientation)
        # Access the position and orientation of the pose
            #input_x, input_y, input_theta =target[0],target[1],theta
            input_x, input_y, input_theta =position.x+dx,position.y+dy,theta
            self.goal_pose_x = input_y
            self.goal_pose_y = input_x
            self.goal_pose_theta = input_theta
            self.get_logger().info(f"Going to {self.goal_pose_x},{self.goal_pose_y} from {self.last_pose_x},{self.last_pose_y} .")
            # Step 1: Turn
            if self.step == 1:
                path_theta = math.atan2(
                    self.goal_pose_y - self.last_pose_y,
                    self.goal_pose_x - self.last_pose_x)
                angle = path_theta - self.last_pose_theta
                angular_velocity = 0.1  # unit: rad/s

                twist, self.step = Turtlebot3Path.turn(angle, angular_velocity, self.step)

            # Step 2: Go Straight
            elif self.step == 2:
                distance = math.sqrt(
                    (self.goal_pose_x - self.last_pose_x)**2 +
                    (self.goal_pose_y - self.last_pose_y)**2)
                linear_velocity = 0.1  # unit: m/s

                twist, self.step = Turtlebot3Path.go_straight(distance, linear_velocity, self.step)

            # Step 3: Turn
            elif self.step == 3:
                angle = self.goal_pose_theta - self.last_pose_theta
                angular_velocity = 0.1  # unit: rad/s

                twist, self.step = Turtlebot3Path.turn(angle, angular_velocity, self.step)

            # Reset
            elif self.step == 4:
                self.step = 1
                self.get_key_state = False

            self.cmd_vel_pub.publish(twist)


    """*******************************************************************************
    ** Below should be replaced when porting for ROS 2 Python tf_conversions is done.
    *******************************************************************************"""
    def euler_from_quaternion(self, quat):
        """
        Convert quaternion (w in last place) to euler roll, pitch, yaw.

        quat = [x, y, z, w]
        """
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = numpy.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
