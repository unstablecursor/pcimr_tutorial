#!/usr/bin/env python
import os
import math
import rospy
import numpy as np
from threading import Lock

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker

class LocalizationNode:

    def __init__(self):
        # Initialize member variables
        self.sim_lock = Lock()

        self.move_sub = rospy.Subscriber('/move', String, self.get_move)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.get_map)
        self.sub_scan_sub = rospy.Subscriber('/scan', LaserScan, self.get_scan)

        # self.max_velocity = rospy.get_param('/controller_node/max_velocity', 0.5)
        # self.stop_distance = rospy.get_param('/controller_node/stop_distance', 0.2)
        # self.slowdown_distance = rospy.get_param('/controller_node/slowdown_distance', 0.4)
        # self.robot_ns = rospy.get_param('/controller_node/robot_namespace', "rto-1")
        self.map_updates_pub = rospy.Publisher('/map_updates', OccupancyGrid, queue_size=10)
        self.pub_twist_pub = rospy.Publisher('/visualization/robot_pos', Marker, queue_size=10)
        self.robot_ros_map_pub  = rospy.Publisher('/robot_pos_map', OccupancyGrid, queue_size=10)

        self.robot_move = String()
        self.map = OccupancyGrid()
             
        self.scan = LaserScan()
        self.robot_twist = Twist()
        self.filtered_twist = Twist()

    def get_move(self, msg):
        self.robot_twist = msg

    def get_map(self, msg):
        self.robot_twist = msg

    def get_scan(self, msg):
        self.robot_twist = msg

    
    def filter_twist(self):
        rospy.loginfo(f"Got twist: {self.robot_twist}")
        self.filtered_twist = self.robot_twist
        if self.robot_twist.linear.x > self.max_velocity:
            self.filtered_twist.linear.x = self.max_velocity
        if self.robot_twist.linear.x > 0.0 :
            min_dist_obs = min(self.scan.ranges)
            if min_dist_obs < self.slowdown_distance:
                val = min_dist_obs - self.stop_distance
                rospy.loginfo(f"Val : {val}, diif {self.slowdown_distance - self.stop_distance} ")
                if val > 0.0:
                    self.filtered_twist.linear.x = self.filtered_twist.linear.x * (val / (self.slowdown_distance - self.stop_distance)) 
                else: 
                    self.filtered_twist.linear.x = 0.0
        else: 
            self.filtered_twist.linear.x = 0.0
        rospy.loginfo(f"Filtered twist: {self.filtered_twist}")
    
    def get_scan(self, msg):
        self.scan = msg

    def run(self, rate: float = 1):
        while not rospy.is_shutdown():
            self.filter_twist()
            self.pub_twist.publish(self.filtered_twist)
            #print(f"Published move \n {self.filtered_twist}")

            if rate:
                rospy.sleep(1/rate)


if __name__ == "__main__":
    rospy.init_node('localization_node')

    simple_sim_node = LocalizationNode()
    simple_sim_node.run(rate=20)
