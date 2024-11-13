#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
import numpy as np

class Turtlebot3HighlevelController:
    def __init__(self, nh):
        self.node_handle = nh
        self.pillar_pos = [0.0, 0.0]
        self.p_vel = 0.8  # Proportional gain for linear velocity
        self.p_ang = 0.2  # Proportional gain for angular velocity

        # Load parameters from config file
        self.p_vel = rospy.get_param("controller_gain", self.p_vel)  # default value if param not set
        self.subscriber_topic = rospy.get_param("subscriber_topic", "scan")  # default topic name
        self.queue_size = rospy.get_param("queue_size", 10)  # default queue size

        # Create subscriber and publisher
        self.subscriber = rospy.Subscriber(self.subscriber_topic, LaserScan, self.laser_callback, queue_size=self.queue_size)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.viz_pub = rospy.Publisher("visualization_marker", Marker, queue_size=0)

        # Initialize marker for pillar visualization
        self.init_pillar_marker()
        rospy.loginfo("TurtleBot3 high-level controller node launched!")



    def viz_pillar(self):
        """
        Visualize pillar position with a marker in RViz
        """
        self.marker.pose.position.x = self.pillar_pos[0]
        self.marker.pose.position.y = self.pillar_pos[1]
        self.marker.pose.position.z = -1.0
        self.viz_pub.publish(self.marker)

    def laser_callback(self, msg):
        cluster_size_threshold = 10
        width_threshold = 0.2
        curvature_threshold = 0.05
        pillar_found = False
        min_dist = float('inf')
        min_index = -1

        for i in range(1, len(msg.ranges) - 1):
            dist = msg.ranges[i]
            if not np.isfinite(dist) or dist <= 0.0:
                continue
            
            prev_dist = msg.ranges[i - 1]
            next_dist = msg.ranges[i + 1]
            curvature = abs(prev_dist - 2 * dist + next_dist)

            if curvature > curvature_threshold:
                cluster_size = 0
                cluster_width = 0.0

                j = i
                while j < len(msg.ranges) and abs(msg.ranges[j] - dist) < width_threshold:
                    cluster_size += 1
                    cluster_width += msg.angle_increment * msg.ranges[j]
                    j += 1

                if cluster_size < cluster_size_threshold and cluster_width < width_threshold and dist < min_dist:
                    pillar_found = True
                    min_dist = dist
                    min_index = i

        if pillar_found and min_index >= 0:
            ang = msg.angle_min + msg.angle_increment * min_index

            self.pillar_pos[0] = min_dist * math.cos(ang)
            self.pillar_pos[1] = min_dist * math.sin(ang)

            rospy.loginfo("Pillar detected at %.2f m and %.2f degrees" % (min_dist, ang * 180.0 / math.pi))
            rospy.loginfo("Pillar's coordinate to Turtlebot is [%f, %f]" % (self.pillar_pos[0], self.pillar_pos[1]))

            # Drive the Husky robot towards the pillar
            self.drive_husky()
        else:
            rospy.loginfo("No pillar detected.")
            
        # 当距离足够接近时，调用停止
        if min_dist < 0.15:
            rospy.signal_shutdown("Pillar is close enough. Shutting down.")  # Stop the node and terminate rospy.spin()
            return 
            
    def drive_husky(self):
        # Combine heading and speed adjustments
        cmd = Twist()
        cmd.linear.x = min(0.5, (self.pillar_pos[0] - 0.2))  # Ensure safe approach
        cmd.angular.z = 0.8 * math.atan2(self.pillar_pos[1], self.pillar_pos[0])
        self.vel_pub.publish(cmd)




    def init_pillar_marker(self):
        """
        Initialize the pillar marker for RViz visualization
        """
        self.marker = Marker()
        self.marker.header.frame_id = "base_laser"
        self.marker.ns = "pillar"
        self.marker.id = 1
        self.marker.type = Marker.CYLINDER
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = self.pillar_pos[0]
        self.marker.pose.position.y = self.pillar_pos[1]
        self.marker.pose.position.z = -1.0
        self.marker.scale.x = 0.5
        self.marker.scale.y = 0.5
        self.marker.scale.z = 2.0
        self.marker.color.a = 1.0  # Alpha transparency
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0

    def update_marker_time(self):
        """
        Update the timestamp of the marker for RViz
        """
        self.marker.header.stamp = rospy.Time.now()


if __name__ == "__main__":
    rospy.init_node("turtlebot3_highlevel_controller")
    controller = Turtlebot3HighlevelController(rospy)
    rospy.spin()
