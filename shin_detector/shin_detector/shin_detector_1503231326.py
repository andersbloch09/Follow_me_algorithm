import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler
from sklearn.neighbors import KNeighborsClassifier
import numpy as np
import cv2
import math

#----------KNN------------------------
# Read in the negative features file
with open('/home/peter/ros2_ws/src/shin_detector/shin_detector/Negative_Features.txt', 'r') as f:
    negative_features = [list(map(np.float64, line.strip().split(','))) for line in f.readlines()]

# Read in the positive features file
with open('/home/peter/ros2_ws/src/shin_detector/shin_detector/Positive_Features.txt', 'r') as f:
    positive_features = [list(map(np.float64, line.strip().split(','))) for line in f.readlines()]

# Combine the positive and negative features into a single array
knnfeatures = np.concatenate([negative_features, positive_features])

# Create the labels for the features (0 for negative, 1 for positive)
knnlabels = np.concatenate([np.zeros(len(negative_features)), np.ones(len(positive_features))])

# Define the number of nearest neighbors to use
k = 3

# Create the K-nearest neighbor classifier
knn = KNeighborsClassifier(n_neighbors=k)

# Train the classifier on the features and labels
knn.fit(knnfeatures, knnlabels)


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

class WaypointPublisher(Node):
    def __init__(self,coordinates):
        self.coordinates = coordinates
        super().__init__('targetCoordinate_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'targetCoordinates', 1)
        timer_period = 0.33  # seconds
        self.timer = self.create_timer(timer_period, self.publish_callback)


    def publish_callback(self):
        msg = Float32MultiArray()
        msg.data = [self.coordinates[0],self.coordinates[1]]
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: ({msg.data})")

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

def getFeatures(clust_x, clust_y, clustersize):

            x_mean = sum(clust_x)/clustersize
            y_mean = sum(clust_y)/clustersize

            clust_x_sorted = np.sort(clust_x)
            clust_y_sorted = np.sort(clust_y)
            x_median = np.median(clust_x_sorted)
            y_median = np.median(clust_y_sorted)

            sum_std_diff = sum_med_diff = 0
            for i in range(clustersize):
                sum_std_diff += pow(clust_x[i]-x_mean, 2) + pow(clust_y[i]-y_mean, 2)
                sum_med_diff += math.sqrt(pow(clust_x[i]-x_median, 2)+pow(clust_y[i] - y_median, 2))

            print(f"FEATURE DEBUG: {clustersize} and {sum_std_diff}")
            std = math.sqrt(1/(clustersize-1)*sum_std_diff)
            
            avg_med_dev = sum_med_diff / clustersize

            #122

            points2=np.vstack((clust_x,clust_y))
            # print('ff',points2.shape)
            points2=np.transpose(points2)

            # print('ff',points2.shape)

            W = np.zeros((2,2), np.float64)
            w = np.zeros((2,2), np.float64)
            U = np.zeros((clustersize, 2), np.float64)

            V = np.zeros((2,2), np.float64)

            w,u,vt=cv2.SVDecomp(points2,W,U,V)
            # print('ww',w,W)
            rot_points = np.zeros((clustersize,2), np.float64)

            W[0,0]=w[0]
            W[1,1]=w[1]
            rot_points = np.matmul(u,W)
            # print(w)
            # print(u)
            # print(vt)
            # print(rot_points.shape)

            linearity=0
            for i in range(clustersize):
                linearity += pow(rot_points[i, 1], 2)


            #Circularity
            A = np.zeros((clustersize,3), np.float64)
            B = np.zeros((clustersize,1), np.float64)


            for i in range(clustersize):
                A[i,0]=-2.0 * clust_x[i]
                A[i,1]=-2.0 * clust_y[i]
                A[i,2]=1
                B[i,0]=math.pow(clust_x[i], 2)-math.pow(clust_y[i], 2)

            sol = np.zeros((3,1),np.float64)
            #cv2.solve(A, B, sol, cv2.DECOMP_SVD)

            xc = sol[0,0]
            yc = sol[1,0]
            rc = math.sqrt(pow(xc, 2)+pow(yc, 2)) - sol[2,0]


            radius = rc #Radius

            features=[clustersize, std, avg_med_dev, linearity, radius]
            return features

def main(args=None):
    rclpy.init(args=args)

    #Make subscriber and lists for x and y coordinates of point cloud
    lidar_subscriber = LIDARsubscriber()
    coord_publisher = WaypointPublisher(coordinates=[0.0,0.0])
    global vectorArrX
    global vectorArrY
    vectorArrX = np.zeros(359)
    vectorArrY = np.zeros(359)
    #Generate the first clustered data, for reference in the loop.
    #genCluster(lidar_subscriber)

    while(1):
        rclpy.spin_once(lidar_subscriber)
        points = np.asarray(lidar_subscriber.dist)
        print(f"Length of Ranges: {len(lidar_subscriber.dist)}")
        positiveClustCoord = []
        for i in range(90):
            vectorArrX[i]=np.cos(np.deg2rad(i+1))*points[i]
            vectorArrY[i]=np.sin(np.deg2rad(i+1))*points[i]

        for i in range(270,359):
            vectorArrX[i]=np.cos(np.deg2rad(i+1))*points[i]
            vectorArrY[i]=np.sin(np.deg2rad(i+1))*points[i]

        coord = tuple(zip(vectorArrX,vectorArrY))
        X = StandardScaler().fit_transform(coord)
        
        #Clustering
        db = DBSCAN(eps=0.10, min_samples=4).fit(coord)

        labels = db.labels_
        label = np.array(labels)
        
        n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
        n_noise_ = list(labels).count(-1)


        print("------------------------------------------------")
        #Extract points in each cluster an extract their features
        for i in range(1, n_clusters_):
            tempX = []
            tempY = []
            #If the cluster contains more than 1 point; complete the code
            if np.count_nonzero(label==i) > 1:
                for j in range(len(label)):
                    if i==label[j]: 
                        tempX.append(vectorArrX[j])
                        tempY.append(vectorArrY[j])
            # Predict the class of the object
            
            features = getFeatures(tempX,tempY,len(tempX))

            predicted_class = knn.predict([features])[0]

            # Output the predicted class
            if predicted_class == 0:
                print(f'The object (Cluster no.:{i}) is classified as negative.')
            else:
                print(f'The object (Cluster no.:{i}) is classified as positive.')
                centerpointX = sum(tempX)/len(tempX)
                centerpointY = sum(tempY)/len(tempY)

                positiveClustCoord.append((centerpointX,centerpointY))

        print(f"Cluster centerpoint coordinates {positiveClustCoord}")


        lengths = []
        for (x,y) in positiveClustCoord:
            lengths.append(np.linalg.norm([x,y]))
        
        for i in range(len(lengths)):
            if min(lengths)==lengths[i]:
                x1,y1 = positiveClustCoord[i]
                coord_publisher.coordinates[0] = x1
                coord_publisher.coordinates[1] = y1
                break
        
        rclpy.spin_once(coord_publisher)
        coord_publisher.coordinates[0] = 0.0
        coord_publisher.coordinates[1] = 0.0

        print(f"clustersize_1: {np.count_nonzero(label==1)}")
        print(f"clustersize_2: {np.count_nonzero(label==2)}")
        # Number of clusters in labels, ignoring noise if present.

        print("Estimated number of clusters: %d" % n_clusters_)
        print("Estimated number of noise points: %d" % n_noise_)
        
    coord_publisher.destroy_node()
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()