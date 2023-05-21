import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler
import matplotlib.pyplot as plt
import numpy as np

#------------CLASSES----------------
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

#-----------FUNCTIONS---------------
def genCluster(subscriber):
    rclpy.spin_once(subscriber)
    points = np.asarray(subscriber.dist)
    for i in range(len(points)):
        vectorArrX[i]=np.cos(np.deg2rad(i))*points[i]
        vectorArrY[i]=np.sin(np.deg2rad(i))*points[i]
    
    coord = tuple(zip(vectorArrX,vectorArrY))
    global db
    db = DBSCAN(eps=0.05, min_samples=5,).fit(coord)


def main(args=None):
    rclpy.init(args=args)

    #Make subscriber and lists for x and y coordinates of point cloud
    lidar_subscriber = LIDARsubscriber()
    global vectorArrX
    global vectorArrY
    vectorArrX = np.zeros(40)
    vectorArrY = np.zeros(40)
    #Generate the first clustered data, for reference in the loop.
    #genCluster(lidar_subscriber)

    while(1):
        rclpy.spin_once(lidar_subscriber)
        points = np.asarray(lidar_subscriber.dist)
        
        for i in range(40):
            vectorArrX[i]=np.cos(np.deg2rad(i))*points[i]
            vectorArrY[i]=np.sin(np.deg2rad(i))*points[i]

        coord = tuple(zip(vectorArrX,vectorArrY))
        X = StandardScaler().fit_transform(coord)
        
        
        db = DBSCAN(eps=0.05, min_samples=5,).fit(coord)
        labels = db.labels_
        print(labels)

        # Number of clusters in labels, ignoring noise if present.
        n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
        n_noise_ = list(labels).count(-1)

        print("Estimated number of clusters: %d" % n_clusters_)
        print("Estimated number of noise points: %d" % n_noise_)
            
        #-----------------
        unique_labels = set(labels)
        core_samples_mask = np.zeros_like(labels, dtype=bool)
        core_samples_mask[db.core_sample_indices_] = True

        colors = [plt.cm.Spectral(each) for each in np.linspace(0, 1, len(unique_labels))]
        for k, col in zip(unique_labels, colors):
            if k == -1:
                # Black used for noise.
                col = [0, 0, 0, 1]

            class_member_mask = labels == k

            xy = X[class_member_mask & core_samples_mask]
            plt.plot(
                xy[:, 0],
                xy[:, 1],
                "o",
                markerfacecolor=tuple(col),
                markeredgecolor="k",
                markersize=14,
            )

            xy = X[class_member_mask & ~core_samples_mask]
            plt.plot(
                xy[:, 0],
                xy[:, 1],
                "o",
                markerfacecolor=tuple(col),
                markeredgecolor="k",
                markersize=6,
            )

        plt.title(f"Estimated number of clusters: {n_clusters_}")
        plt.show()

    
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()
