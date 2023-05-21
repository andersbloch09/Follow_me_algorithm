import os
import select
import sys
import rclpy
import numpy as np
import math
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from math import pi
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.qos import ReliabilityPolicy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from collections import deque


#Firstly choosing operating system: 
import termios
import tty

#-----------------Control Constants----------------
BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84/2

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
message = """The robot will drive now"""
error = """AN ERROR HAS ACCURED"""

# Deque size limit
queueLimit = 16


def make_simple_profile(output, input, step):
    if input > output:
        output = min(input, output + step)
    elif input < output:
        output = max(input, output - step)
    else:
        output = input
    return output

def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel

def check_linear_limit_velocity(velocity):
    return constrain(velocity, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

def check_angular_limit_velocity(velocity):
    return constrain(velocity, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

class Imu_Sub(Node):
    def __init__(self):
        super().__init__('Imu_Sub')  #This name i call the same as the node name
        self.subscription = self.create_subscription(
            Imu,
            'imu',
            self.listener_callback,
            10)#First the msg type, topic, callback_function, and lastly our QoS(quality of service i think)
        self.subscription 

    def listener_callback(self, msg: Imu):#We have our message AKA data 
        self.orientation = quaternion_to_radians(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        
class target_Sub(Node): #Subscriber that extracts the waypoints that it's supposed to follow
    def __init__(self):
        super().__init__('target_Sub')  #This name i call the same as the node name
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'targetCoordinates',
            self.listener_callback,
            10)#First the msg type, topic, callback_function, and lastly our QoS(quality of service i think)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: Float32MultiArray):#We have our message AKA data 
        self.x = msg.data[0]
        self.y = msg.data[1]

class Point_Sub(Node): #Subscriber that extracts global position data of the TB3
    def __init__(self):
        super().__init__('Point_Sub')  #This name i call the same as the node name
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.listener_callback,
            10)#First the msg type, topic, callback_function, and lastly our QoS(quality of service i think)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: Odometry):#We have our message AKA data 
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

class LIDAR_Sub(Node): #Subscriber that extracts lidar range data
    def __init__(self):
        super().__init__('LIDAR_Sub')  #This name i call the same as the node name
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))#First the msg type, topic, callback_function, and lastly our QoS(quality of service i think)
        self.subscription  # prevent unused variable warning
        self.dist = np.zeros(360, dtype=float)
        
    def listener_callback(self, msg: LaserScan):#We have our message AKA data 
        self.dist = msg.ranges
        
#Takes quaternions from the IMU and yields the current orientation about the z-axis in radians
def quaternion_to_radians(w, x, y, z):
    # Calculate the yaw angle
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))

    return yaw

def transform_to_global_frame(x_l, y_l, robot_x, robot_y, rotation_z):
    # Apply the transformation
    x_g = robot_x + (x_l * np.cos(rotation_z)) - (y_l * np.sin(rotation_z))
    y_g = robot_y + (x_l * np.sin(rotation_z)) + (y_l * np.cos(rotation_z))

    # Return the transformed point in the global frame
    return [x_g, y_g]

def reg_ang(target_x, target_y, orientation, robotx, roboty): #FIX FIX FIX
                            # Constants 
    k       = 1.58          # Regulation constant 
    pD      = 100/np.deg2rad(180)       # Percentage of degrees - Indicated that when the reference angle is 180, the robot will turn at full speed
    avP     = 0.025         # One percent of max angle velocity in rads

    # Find the angle from the origin to the waypoint in rads
    ang_global = math.atan2(target_y - roboty, target_x - robotx)

    # Determine discrepancy between the goal angle and current angle
    err = ang_global - orientation

    print(f"Error reg_ang: {err}")

    angVelRes = 0.0 #Angular Velocity Result that is sent to the motors.
    
    # If the angle discrepancy is in the interval [-5;5], then the regulatory variable is set to zero
    if np.deg2rad(3) >= err >= np.deg2rad(-3):
        angVelRes = 0.0

    # Inp.deg2rad(f the angle discrepancy is in the interval (0;180), the robot should turn right
    else:
        #go right
        #u       = k*e           #Upscaled degree difference
        #pF      = u*pD          #Percentage boost
        #angVelRes    = avP*pF       #Change in angle velocity 

        angVelRes = k*err*pD*avP

    if angVelRes > 3*np.pi/2:
        angVelRes = -angVelRes + 2*np.pi
    elif angVelRes < -3*np.pi/2:
        angVelRes = -angVelRes - 2*np.pi
    
    return constrain(angVelRes,-BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL) 

def reg_vel(target_x, target_y, robotx, roboty): # FIX FIX FIX
    #This is made if the distance is 0.5 - 2 meters where the ideal length is 1.25.
    #For the none derived version check maple document. 
    standard_vel = 0.11 

    dist = np.linalg.norm([target_x-robotx,target_y-roboty])

    e = -0.2 + dist
    sc = 0.1476851852*e+standard_vel

    return constrain(sc,-BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

def main():
    # Initialize rclpy and create the twist data-class publisher
    rclpy.init()
    node = rclpy.create_node('drive')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    
    # Instantiate all relevant nodes
    target_node = target_Sub()
    Imu_node = Imu_Sub()
    pose_node = Point_Sub()

    #Set all values to 0.0 in setup
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0

    # Make a double ended queue for waypoints
    queue = deque()
    
    # Instantiate the twist data-class (Which is published with new movement data to make the robot move)
    twist = Twist()
    queue.append([0.5,0])
    queue.append([1,0.5])
    queue.append([1,1])
    queue.append([0.5,1.5])
    queue.append([0,1.5])
    queue.append([-0.5,1])
    queue.append([-0.5,0.5])
    queue.append([0,0])
    #------------------
    queue.append([0.5,0])
    queue.append([1,0.5])
    queue.append([1,1])
    queue.append([0.5,1.5])
    queue.append([0,1.5])
    queue.append([-0.5,1])
    queue.append([-0.5,0.5])
    queue.append([0,0])

    print("Entering First Loop")
    while True: 
        # Fill all subscriber nodes with live data
        rclpy.spin_once(pose_node)
        rclpy.spin_once(target_node)
        rclpy.spin_once(Imu_node)
        
        # Put new waypoint into the queue : Check whether the point is proper
        #if target_node.x == 0.0:
        #    print("Waypoint not added to queue: Invalid")
        #else:
        #    waypoint = transform_to_global_frame(target_node.x,target_node.y,pose_node.x,pose_node.y,Imu_node.orientation)
        #    queue.append([waypoint[0],waypoint[1]])
        #
        ## If the queue is empty, update nodes and add valid points until the queue is no longer empty
        #while len(queue) == 0:
        #    rclpy.spin_once(pose_node)
        #    rclpy.spin_once(target_node)
        #    rclpy.spin_once(Imu_node)
        #    
        #    if target_node.x == 0.0:
        #        print("Queue Empty: Invalid Waypoint not added to queue")
        #    else:
        #        waypoint = transform_to_global_frame(target_node.x,target_node.y,pose_node.x,pose_node.y,Imu_node.orientation)
        #        queue.append([waypoint[0],waypoint[1]])



        # Pop first waypoint from queue once, then enter loop to satisfy while loop break condition
        currentWP = queue.popleft()
        distToWP = 1

        print("Entering Second Loop")

        #Break loop when the robot is 25 cm from the current waypoint:
        while distToWP > 0.10:
            print("Driving to point")
            # Continue to update waypoint queue as often as possible
            rclpy.spin_once(pose_node)
            rclpy.spin_once(target_node)
            rclpy.spin_once(Imu_node)

            #if target_node.x == 0.0:
            #    print("Invalid Waypoint in queue")
            #elif np.linalg.norm([currentWP[0]-pose_node.x, currentWP[1]-pose_node.y])< 2: #If the distance to the waypoint is less than 2 meters; append
            #    waypoint = transform_to_global_frame(target_node.x,target_node.y,pose_node.x,pose_node.y,Imu_node.orientation) #Change coordinates to global before appending
            #    queue.append([waypoint[0],waypoint[1]])

            # If more than 100 waypoints exist, remove the first one in the queue until its below the limit
            while len(queue) > queueLimit:
                print("Queue Too Long")
                queue.popleft()

            # Extract last waypoint in the queue and pass it to the lin-vel function:
            # Remove the last point and save it for use, then immediately append it to the back of the queue again (Because deques i guess)
            # Unless queue is empty
            if len(queue) == 0:
                lastWP = currentWP
            else:
                lastWP = queue.popleft() #REMEMBER TO CHANGE BACK TO POP
                queue.appendleft(lastWP)

            # Calculate the distance to the current waypoint to break loop:
            distToWP = np.linalg.norm([currentWP[0]-pose_node.x, currentWP[1]-pose_node.y])

            # Debugging
            print(f"Distance to WP: {distToWP}")
            print(f"Current Waypoint: {currentWP[0]}, {currentWP[1]}")
            print(f"Current Position: {pose_node.x}, {pose_node.y}")
            print(f"Current Orientation: {Imu_node.orientation}")

            # Calculate the Angular and Linear velocities are needed to reach current waypoint
            control_linear_velocity = reg_vel(lastWP[0], lastWP[1],pose_node.x,pose_node.y)
            control_angular_velocity = reg_ang(currentWP[0], currentWP[1], Imu_node.orientation, pose_node.x, pose_node.y)

            # Make linear velocity profile
            control_linear_velocity = make_simple_profile(
                control_linear_velocity,
                target_linear_velocity,
                (LIN_VEL_STEP_SIZE / 2.0))

            twist.linear.x = control_linear_velocity/2

            # Make angular velocity profile
            control_angular_velocity = make_simple_profile(
                control_angular_velocity,
                target_angular_velocity,
                (ANG_VEL_STEP_SIZE / 2.0))

            # Set angular velocity along z-axis
            twist.angular.z = math.atan2(math.sin(control_angular_velocity), math.cos(control_angular_velocity))

            print(f"Control_Angular_velocity: {control_angular_velocity}")
            # Publish all control data
            pub.publish(twist)
            print(f"Publishing Twist: {twist}")

            # Pop first waypoint from queue, continue to feed it to regulation systems until distance to point is X;
            # Keep filling waypoint queue in the meantime
            # When the current point has been reached, pop next in queue.
            # Speed should rely on the distance to the last point in the queue, without removing it
            
    target_node.destroy_node()
    Imu_node.destroy_node()
    pose_node.destroy_node()
    rclpy.shutdown()       

        

if __name__ == '__main__':
    main()