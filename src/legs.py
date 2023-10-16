#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from sklearn.cluster import DBSCAN
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np

class LegDetector:
    def __init__(self):
        rospy.init_node('leg_detector')
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

    def publish_leg_tf_frame(self, x, y):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'Lidar_link'  # Replace with your LIDAR frame ID
        transform.child_frame_id = 'detected_legs_frame'
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.w = 1.0  # No rotation for simplicity

        self.tf_broadcaster.sendTransform(transform)

    def laser_callback(self, msg):
        # Convert LIDAR data to numpy array
        lidar_data = np.array(msg.ranges)

        # Remove invalid measurements (inf and NaN values)
        valid_indices = np.isfinite(lidar_data)
        lidar_data = lidar_data[valid_indices]

        if len(lidar_data) == 0:
            return  # No valid LIDAR measurements, nothing to process

        # Calculate angles corresponding to valid LIDAR measurements
        min_angle = msg.angle_min
        angle_increment = msg.angle_increment
        #print(angle_increment)
        angles = np.arange(min_angle, min_angle + len(lidar_data) * angle_increment, angle_increment)

        # Convert polar coordinates to Cartesian coordinates
        x = lidar_data * np.cos(angles)
        y = lidar_data * np.sin(angles)

        # Stack x and y to form a 2D array
        points = np.vstack((x, y)).T

        # Apply DBSCAN clustering
        dbscan = DBSCAN(eps=0.1, min_samples=5)  # Adjust eps and min_samples as needed
        labels = dbscan.fit_predict(points)

        # Get unique cluster labels
        unique_labels = np.unique(labels)

        # Print detected legs
        # Print detected legs and publish TF frames
        for label in unique_labels:
            if label != -1:  # Exclude noise points
                cluster_points = points[labels == label]
                mean_x = np.mean(cluster_points[:, 0])
                mean_y = np.mean(cluster_points[:, 1])
                print(f"Detected leg at X: {mean_x:.2f}, Y: {mean_y:.2f}")
                
                # Publish TF frame for detected legs
                self.publish_leg_tf_frame(mean_x, mean_y)

if __name__ == '__main__':
    leg_detector = LegDetector()
    rospy.spin()