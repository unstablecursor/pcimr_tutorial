#!/usr/bin/env python
import os
import math
import rospy
import numpy as np
from threading import Lock


from std_msgs.msg import String, ColorRGBA, Header
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Twist, Pose, Quaternion, Vector3, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData, Path
from visualization_msgs.msg import Marker

HEIGHT = 20
WIDTH = 20

class GlobalPlannerNode:

    def __init__(self):
        # Initialize member variables
        self.sim_lock = Lock()

        self.robot_pos_sub = rospy.Subscriber('/robot_pos', Point, self.get_robot_pos)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.get_map)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.get_goal)

        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=10, latch=True)
        self.goal_viz_pub = rospy.Publisher('/visualization/goal', Marker, queue_size=10)
        self.global_path_viz_pub  = rospy.Publisher('/visualization/global_path', Marker, queue_size=10)


        self.robot_pos=Point()
        self.map=OccupancyGrid()
        self.goal=PoseStamped()

        self.global_path = Path()
        self.global_path_viz = Marker()
        self.goal_viz = Marker(header = Header(frame_id='map'),
                                    pose = Pose(Point(0,0,0), Quaternion(0.0,0.0,0.0,1.0)), 
                                    type=1,
                                    scale=Vector3(1,1,0.2),
                                    color=ColorRGBA(0.0, 1.0, 0.0, 1.0))

        self.init_oldmap = False


    def filter_map(self):
        np_map = self.map_to_np(self.map)
        # Create map for publishing
        # Change * 100 to something else to change visuals. However, i think this works best
        pubb_map = pubb_map * 100 
        pubb_map[np_map == -1] = -1
        pubb_map[np_map == 100] = 100
        robot_pos_map.data = self.np_to_map(pubb_map)

    def get_goal(self, msg):
        self.goal =  msg

    def get_map(self, msg):
        self.map = msg

    def get_robot_pos(self, msg):
        self.robot_pos = msg 

    def map_to_np(self, map: OccupancyGrid):
        np_map = np.transpose(np.asarray(map.data, dtype=np.int8).reshape(map.info.width, map.info.height))
        return np_map

    def np_to_map(self, arr: np.array):
        arr = np.array(np.transpose(arr), dtype=np.int8)
        return arr.ravel()  

    def run(self, rate: float = 1):

        while not rospy.is_shutdown():

            self.global_path_pub.publish(self.global_path)
            #rospy.logdebug(f"Sent robot_pos : \n{self.robot_pos}")
            self.goal_viz_pub.publish(self.goal_viz)
            #rospy.logdebug(f"Sent robot_pos_viz : \n{self.robot_pos_viz}")
            self.global_path_viz_pub.publish(self.global_path_viz)
            #rospy.loginfo(f"Sent robot_pos_map : \n{self.robot_pos_map.data}")

            if rate:
                rospy.sleep(1/rate)


if __name__ == "__main__":
    rospy.init_node('global_planner_node')

    node = GlobalPlannerNode()
    node.run(rate=2)