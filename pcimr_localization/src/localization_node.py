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

HEIGHT = 20
WIDTH = 20

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
        self.robot_x = 3
        self.robot_y = 3
        self.robot_pos = Point(self.robot_x,self.robot_y,0)
        self.robot_pos_viz = Marker(header = Header(frame_id='map'),
                                    pose = Pose(Point(self.robot_x+0.5,self.robot_y+0.5,0), Quaternion(0.0,0.0,0.0,1.0)), 
                                    type=1,
                                    scale=Vector3(1,1,0.2),
                                    color=ColorRGBA(0.1, 0.1, 1.0, 1.0))
        self.robot_pos_map = OccupancyGrid(header = Header(frame_id='map', seq=0))
        self.robot_pos_map.info.width = WIDTH
        self.robot_pos_map.info.height = HEIGHT
        self.robot_pos_map.info.resolution = 1

        self.probs_mv = rospy.get_param('/simple_sim_node/robot_move_probabilities', [0.9, 0.04, 0.04, 0.0, 0.02])

        self.np_map = np.zeros((WIDTH, HEIGHT))
        self.oldmap = np.zeros((WIDTH, HEIGHT))
        self.new_move_prob_map = np.zeros((WIDTH, HEIGHT))
        self.new_sensor_prob_map = np.zeros((WIDTH, HEIGHT))
        self.sensor_kernel = np.zeros((WIDTH *2, HEIGHT*2))
        self.init_oldmap = False

        # it's rotated by 90 since our np representation is rotated
        self.move_kernel = np.array([
            [0, self.probs_mv[1], 0],
            [self.probs_mv[3], self.probs_mv[4], self.probs_mv[0]],
            [0, self.probs_mv[2], 0]]
            )

    def convolve_move(self):
        self.new_sensor_prob_map = np.ones((WIDTH, HEIGHT))
        extended_map = np.zeros((WIDTH+2, HEIGHT+2))
        extended_map[1:WIDTH+1, 1:HEIGHT+1] = np.copy(self.oldmap)

        extended_map_sens = np.ones((WIDTH+2, HEIGHT+2))
        prob_arr = [0.1,0.8,0.1]

        for i in range(1,WIDTH+1):
            for j in range(1,HEIGHT+1):
                if self.np_map[i-1,j-1] == 100 or self.np_map[i-1,j-1] == -1:
                    self.new_move_prob_map[i-1,j-1] = 0.0
                else:
                    # Get move probablities
                    self.new_move_prob_map[i-1,j-1] = np.sum(np.multiply(extended_map[i-1:i+2,j-1:j+2], self.move_conv_matrix))
                    if j + self.scan.ranges[2] < HEIGHT:
                        if self.np_map[i-1, j-1 + int(self.scan.ranges[2])] == 100:
                            extended_map_sens[i, j-1:j+2] *= prob_arr
                    if j - self.scan.ranges[0] < HEIGHT:
                        if self.np_map[i-1, j-1 - int(self.scan.ranges[0])] == 100:
                            extended_map_sens[i, j-1:j+2] *= prob_arr
                    if i + self.scan.ranges[3] < WIDTH:
                        if self.np_map[i-1 + int(self.scan.ranges[3]), j-1] == 100:
                            extended_map_sens[i-1:i+2,j] *= prob_arr
                    if i - self.scan.ranges[1] < WIDTH:
                        if self.np_map[i-1 - int(self.scan.ranges[1]), j-1] == 100:
                            extended_map_sens[i-1:i+2,j] *= prob_arr


        self.new_sensor_prob_map = np.copy(extended_map_sens[1:WIDTH+1, 1:HEIGHT+1])
        self.new_sensor_prob_map[self.new_sensor_prob_map == 1] = 0.0
        self.new_sensor_prob_map[self.np_map == -1] = 0.0
        self.new_sensor_prob_map[self.np_map == 100] = 0.0
        inter_map = np.multiply(self.new_sensor_prob_map, self.new_move_prob_map)
        normalization = 1.0 / np.sum(inter_map)
        self.new_move_prob_map = np.copy(inter_map * normalization)
        self.oldmap = np.copy(self.new_move_prob_map)
        

    def filter_map(self):
        np_map = self.map_to_np(self.map)
        #rospy.loginfo(np_map)
        # TODO: Filter map here...
        pubb_map = np.copy(self.new_move_prob_map)
        pubb_map = pubb_map * 100 
        pubb_map[np_map == -1] = -1
        pubb_map[np_map == 100] = 100
        rospy.loginfo(pubb_map)
        self.robot_pos_map.data = self.np_to_map(pubb_map)

    def find_and_update_robot_pos(self):
        t = np.argmax(self.new_move_prob_map)
        self.robot_x = t // WIDTH
        self.robot_y = t % HEIGHT

    def get_move(self, msg):
        if msg.data == "N":
            self.move_conv_matrix = np.rot90(self.move_kernel, 2)
        if msg.data == "S":
            self.move_conv_matrix = np.rot90(self.move_kernel, 4)
        if msg.data == "E":
            self.move_conv_matrix = np.rot90(self.move_kernel, 1)
        if msg.data == "W":
            self.move_conv_matrix = np.rot90(self.move_kernel, 3)
        self.robot_move = msg
        if self.init_oldmap:
            self.convolve_move()
            self.find_and_update_robot_pos()
            self.filter_map()

    def get_map(self, msg):
        #rospy.logdebug(f"Got map : \n{msg}")
        self.map = msg
        if not self.init_oldmap:
            self.np_map = self.map_to_np(self.map)
            self.init_oldmap = True


    def get_scan(self, msg):
        self.scan = msg 

    def map_to_np(self, map: OccupancyGrid):
        np_map = np.transpose(np.asarray(map.data, dtype=np.int8).reshape(map.info.width, map.info.height))
        return np_map

    def np_to_map(self, arr: np.array):
        arr = np.array(np.transpose(arr), dtype=np.int8)
        return arr.ravel()
    

        

    def run(self, rate: float = 1):
        self.oldmap[self.robot_x][self.robot_y] = 1
        self.robot_pos_pub.publish(self.robot_pos)

        while not rospy.is_shutdown():
            self.robot_pos = Point(self.robot_x,self.robot_y,0)
            self.robot_pos_viz.pose.position.x = self.robot_x + 0.5
            self.robot_pos_viz.pose.position.y = self.robot_y + 0.5

            self.robot_pos_pub.publish(self.robot_pos)
            #rospy.logdebug(f"Sent robot_pos : \n{self.robot_pos}")
            self.robot_pos_viz_pub.publish(self.robot_pos_viz)
            #rospy.logdebug(f"Sent robot_pos_viz : \n{self.robot_pos_viz}")
            self.robot_ros_map_pub.publish(self.robot_pos_map)
            #rospy.loginfo(f"Sent robot_pos_map : \n{self.robot_pos_map.data}")

            if rate:
                rospy.sleep(1/rate)


if __name__ == "__main__":
    rospy.init_node('localization_node')

    node = LocalizationNode()
    node.run(rate=2)
