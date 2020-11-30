#!/usr/bin/env python
import os
import math
import rospy
import numpy as np
from threading import Lock

from std_msgs.msg import String, ColorRGBA, Header
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Twist, Pose, Quaternion, Vector3
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker

class LocalizationNode:

    def __init__(self):
        # Initialize member variables
        self.sim_lock = Lock()

        self.move_sub = rospy.Subscriber('/move', String, self.get_move)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.get_map)
        self.sub_scan_sub = rospy.Subscriber('/scan', LaserScan, self.get_scan)

        self.robot_pos_pub = rospy.Publisher('/robot_pos', Point, queue_size=10)
        self.robot_pos_viz_pub = rospy.Publisher('/visualization/robot_pos', Marker, queue_size=10)
        self.robot_ros_map_pub  = rospy.Publisher('/robot_pos_map', OccupancyGrid, queue_size=10)

        self.robot_move = String()
        self.map = OccupancyGrid()
        self.scan = LaserScan()

        self.robot_pos = Point(3,3,0)
        self.robot_pos_viz = Marker(header = Header(frame_id='map'),
                                    pose = Pose(self.robot_pos, Quaternion(0.0,0.0,0.0,1.0)), 
                                    type=1,
                                    scale=Vector3(1,1,1),
                                    color=ColorRGBA(0.1, 0.1, 1.0, 1.0))
        self.robot_pos_map = OccupancyGrid()

    def get_move(self, msg):
        rospy.logdebug(f"Got move : \n{msg}")
        self.robot_move = msg

    def get_map(self, msg):
        rospy.logdebug(f"Got map : \n{msg}")
        self.map = msg

    def get_scan(self, msg):
        rospy.logdebug(f"Got scan : \n{msg}")
        self.scan = msg

    def map_to_np(self, map: OccupancyGrid):
        np_map = np.transpose(np.asarray(map.data, dtype=np.int8).reshape(map.info.width, map.info.height))
        return np_map

    def np_to_map(self, arr: np.array):
        arr = np.array(np.transpose(arr), dtype=np.int8)
        grid = OccupancyGrid()
        grid.data = arr.ravel()
        grid.info = MapMetaData()
        grid.info.height = arr.shape[0]
        grid.info.width = arr.shape[1]

        return grid
    
    def filter_map(self):
        np_map = self.map_to_np(self.map)
        # TODO: Filter map here...
        self.robot_pos_map = self.np_to_map(np_map)
        

    def run(self, rate: float = 1):
        while not rospy.is_shutdown():
            self.filter_map()
            self.robot_pos_pub.publish(self.robot_pos)
            rospy.logdebug(f"Sent robot_pos : \n{self.robot_pos}")
            self.robot_pos_viz_pub.publish(self.robot_pos_viz)
            rospy.logdebug(f"Sent robot_pos_viz : \n{self.robot_pos_viz}")
            self.robot_ros_map_pub.publish(self.robot_pos_map)
            rospy.logdebug(f"Sent robot_pos_map : \n{self.robot_pos_map}")

            if rate:
                rospy.sleep(1/rate)


if __name__ == "__main__":
    rospy.init_node('localization_node')

    node = LocalizationNode()
    node.run(rate=10)
