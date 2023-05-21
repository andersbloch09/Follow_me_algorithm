import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import LaserScan
import time

class LIDARsubscriber(Node):
    def __init__(self):
        super().__init__("LIDAR_subscriber")
        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.listener_callback,
            QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscription
    
    def listener_callback(self,msg:LaserScan):
        self.dist = msg.ranges

def main(args=None):
    rclpy.init(args=args)

    lidar_subscriber = LIDARsubscriber()
    i = 0
    change_time=0
    while(1):
        begin_time=time.time()
        rclpy.spin_once(lidar_subscriber)
        print(lidar_subscriber.dist[0])
        change_time += time.time()-begin_time
        i += 1
        if change_time > 1:
            print(f"Cycle time: {change_time}")
            print(f"Number of samples in cycle: {i}")
            change_time=0
            i=0
            


    
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
