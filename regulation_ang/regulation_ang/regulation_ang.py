import os
import select
import sys
import rclpy
import numpy as np
import time
import array as arr
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from math import pi
from scipy.spatial.transform import Rotation as R

#Firstly choosing operating system: 
import termios
import tty

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

message = """The robot will drive now"""

e = """AN ERROR HAS ACCURED"""

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_vels(target_linear_velocity, target_angular_velocity):
    print('currently:\tlinear velocity {0}\t angular velocity {1} '.format(
        target_linear_velocity,
        target_angular_velocity))

def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
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
        self.subscription  # prevent unused variable warning
        self.imu_ang_vel = float()#We use this data type for the variable as in the Imu message google documentation of Imu message
        self.imu_ang_vel_deg = float()#Same here
        self.imu_ori = Quaternion()
        self.imu_ori_qw = float()
        self.imu_ori_qx = float()
        self.imu_ori_qy = float()
        self.imu_ori_qz = float()

    def listener_callback(self, msg: Imu):#We have our message AKA data 
        self.imu_ang_vel = msg.angular_velocity.z
        self.imu_ang_vel_deg = 180/pi*msg.angular_velocity.z
        self.imu_ori_qw = msg.orientation.w
        self.imu_ori_qx = msg.orientation.x
        self.imu_ori_qy = msg.orientation.y
        self.imu_ori_qz = msg.orientation.z

def quaternions_to_euler(qx, qy, qz, qw):
    if (qx + qy + qz + qw) != 0:
        r = R.from_quat([qx, qy, qz, qw])
        euler=r.as_euler('xyz', degrees=False)
        imu_z = euler[2]*(180/pi)
        print(imu_z)
    return imu_z

def main():
    settings = None
    if os.name != 'nt':#This desides our operating system 
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('drive')#The node is created 
    pub = node.create_publisher(Twist, 'cmd_vel', qos)#The publisher to /cmd_vel

    Imu_node = Imu_Sub()

    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0

    start_time = time.time()#The first timer is started 
    angs = []
    times = []
    i = 0
    j = 0
    target_angular_velocity = 0.1#Angular speed is set to 0.1 rad/s 
    while True: 
        
        second_time = time.time()

        print(Imu_node.imu_ang_vel)

        real_vel = Imu_node.imu_ang_vel#The real value is measured from the IMU sub 

        if second_time - start_time > 3 and target_angular_velocity != 0.00:#After 3 seconds and the velocity is different from 0 
            target_angular_velocity = 0.2#The step is measured as from 0.1 to 0.2 rad/s 
            if i == 0: 
                third_time = time.time()#Start measure time is made 
            fourth_time = time.time()
            measure_time = fourth_time - third_time#Measure time is made
            angs.append(real_vel)#The velocities are appended to array 
            times.append(measure_time)#Times are appeneded to array 
            print("VELOCITY  = " + str(real_vel) + "   TIME   = " + str(measure_time))
            i += 1

        if second_time - start_time > 3.5: #After a 0.5 
            target_angular_velocity = 0.00#Stops the robot 
            control_angular_velocity = 0.00
            print("Done!")

            if real_vel < 0.2:
                np.savetxt("ang_data_10", np.column_stack((times, angs)), fmt=('%.5f','%.5f'))#Points are saved in txt file
                plt.plot(times, angs)#The points are plotted 
                plt.show()
                exit()

        twist = Twist()

        control_linear_velocity = make_simple_profile(
            control_linear_velocity,
            target_linear_velocity,
            (LIN_VEL_STEP_SIZE / 2.0))

        twist.linear.x = control_linear_velocity
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        control_angular_velocity = make_simple_profile(
            control_angular_velocity,
            target_angular_velocity,
            (ANG_VEL_STEP_SIZE / 2.0))

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = control_angular_velocity
        pub.publish(twist)
        rclpy.spin_once(Imu_node)

if __name__ == '__main__':
    main()
