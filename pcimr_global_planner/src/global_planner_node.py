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

        self.publish = False

        self.robot_pos=Point()
        self.map=OccupancyGrid()
        self.goal=PoseStamped()

        self.global_path = Path(header = Header(frame_id='map'))
        self.global_path_viz = Marker(header = Header(frame_id='map'),
                                    pose = Pose(Point(0,0,0), Quaternion(0.0,0.0,0.0,1.0)), 
                                    type=4,
                                    scale=Vector3(0.4,0.0,0.0),
                                    color=ColorRGBA(0.0, 1.0, 0.0, 1.0))
        self.goal_viz = Marker(header = Header(frame_id='map'),
                                    pose = Pose(Point(0,0,0), Quaternion(0.0,0.0,0.0,1.0)), 
                                    type=1,
                                    scale=Vector3(0.8,0.8,0.2),
                                    color=ColorRGBA(1.0, 0.0, 0.0, 1.0))

        self.init_oldmap = False


    # def filter_map(self):
    #     np_map = self.map_to_np(self.map)
    #     # Create map for publishing
    #     # Change * 100 to something else to change visuals. However, i think this works best
    #     pubb_map = pubb_map * 100 
    #     pubb_map[np_map == -1] = -1
    #     pubb_map[np_map == 100] = 100
    #     robot_pos_map.data = self.np_to_map(pubb_map)
    def map_to_np(self, map: OccupancyGrid):
        np_map = np.transpose(np.asarray(map.data, dtype=np.int8).reshape(map.info.width, map.info.height))
        return np_map

    def np_to_map(self, arr: np.array):
        arr = np.array(np.transpose(arr), dtype=np.int8)
        return arr.ravel()  

    def get_map(self, msg):
        #rospy.loginfo(f"Got map: {msg}")
        self.map = msg

    def get_robot_pos(self, msg):
        #rospy.loginfo(f"Got robot pos: {msg}")
        self.robot_pos = msg 

    def get_goal(self, msg):
        #rospy.loginfo(f"Got goal: {msg}")
        self.goal =  msg
        self.update_goal_marker(msg.pose.position.x, msg.pose.position.y)
        path_as_list = self.run_a_star()
        if path_as_list is not None:
            self.update_path(path_as_list)
            self.update_path_marker(path_as_list)
            self.publish = True
        # TODO: Visualize goal, Update costmap once, check whether goal has given already. 


    def get_heuristic(self, x, y):
        return abs(self.goal.pose.position.x - x) ** 2 + abs(self.goal.pose.position.y - y) ** 2
    
    def update_path_marker(self, l):
        self.global_path_viz = Marker(header = Header(frame_id='map'),
                            pose = Pose(Point(0,0,0), Quaternion(0.0,0.0,0.0,1.0)), 
                            type=4,
                            scale=Vector3(0.1,0.0,0.0),
                            color=ColorRGBA(1.0, 1.0, 0.0, 1.0))
        for item in l:
            self.global_path_viz.points.append(Point(item[0]+0.5, item[1]+0.5, 0))
    
    def update_path(self, l):
        self.global_path = Path(header = Header(frame_id='map'))
        for item in l:
            pose = PoseStamped(header = Header(frame_id='map'), pose=Pose(Point(item[0],item[1],0), Quaternion(0.0,0.0,0.0,1.0)))
            self.global_path.poses.append(pose)
    
    def update_goal_marker(self, x, y):
        self.goal_viz.pose.position.x = int(x) + 0.5
        self.goal_viz.pose.position.y = int(y) + 0.5

    def get_path_parents(self, goal_x, goal_y, parents):
        x = goal_x
        y = goal_y
        path = []
        path.insert(0, [x, y])
        while x != int(self.robot_pos.x) or y != int(self.robot_pos.y):
            x_ = parents[x,y][0]
            y_ = parents[x,y][1]
            path.insert(0, [parents[x,y][0], parents[x,y][1]])
            x = x_
            y = y_
        print(path)
        return path

    def run_a_star(self):
        print(self.robot_pos.x, int(self.robot_pos.x))
        x = int(self.robot_pos.x)
        y = int(self.robot_pos.y)
        goal_x = int(self.goal.pose.position.x)
        goal_y = int(self.goal.pose.position.y)
        np_map = self.map_to_np(self.map)
        parents = np.zeros((np_map.shape[0], np_map.shape[1], 2), dtype=np.int32)
        is_visited = np.zeros(np_map.shape, dtype=np.int32)
        # Set unreachable to -1 to prevent visiting them later
        is_visited[np_map == 100] = -1
        is_visited[np_map == -1] = -1
        if is_visited[goal_x, goal_y] == -1:
            rospy.logwarn("Goal is unfeasible!")
            return None
        cost_g = np.full(np_map.shape, np.inf)
        cost_g[x,y] = 0
        costs = np.full(np_map.shape, np.inf)
        costs[x,y] = self.get_heuristic(x,y) + cost_g[x,y]
        path = []
        while 0 in is_visited:
            curr_x = np.where(np.logical_and(is_visited == 0, costs == costs.min()))[0][0]
            curr_y = np.where(np.logical_and(is_visited == 0, costs == costs.min()))[1][0]
            print(curr_x, curr_y)
            if curr_x == goal_x and curr_y == goal_y:
                print("Found goal!")
                path = self.get_path_parents(goal_x, goal_y, parents)
                print(f"Path is {path}")
                break
            # Update neighbours and set parents
            if curr_x != 0:
                if is_visited[curr_x-1,curr_y] == 0:
                    cost_g[curr_x-1,curr_y] = cost_g[curr_x,curr_y] + 1
                    costs[curr_x-1,curr_y] = self.get_heuristic(curr_x-1,curr_y) + cost_g[curr_x-1,curr_y]
                    parents[curr_x-1,curr_y] = [curr_x,curr_y]

            if curr_y != 0: 
                if is_visited[curr_x,curr_y-1]  == 0:
                    cost_g[curr_x,curr_y-1] = cost_g[curr_x,curr_y] + 1
                    costs[curr_x,curr_y-1] = self.get_heuristic(curr_x,curr_y-1) + cost_g[curr_x,curr_y-1]
                    parents[curr_x,curr_y-1] = [curr_x,curr_y]

            if curr_x != np_map.shape[0] - 1:
                if is_visited[curr_x+1,curr_y]  == 0:
                    cost_g[curr_x+1,curr_y] = cost_g[curr_x,curr_y] + 1
                    costs[curr_x+1,curr_y] = self.get_heuristic(curr_x+1,curr_y) + cost_g[curr_x+1,curr_y]
                    parents[curr_x+1,curr_y] = [curr_x,curr_y]

            if curr_y != np_map.shape[1] - 1:
                if is_visited[curr_x,curr_y+1]  == 0:
                    cost_g[curr_x,curr_y+1] = cost_g[curr_x,curr_y] + 1
                    costs[curr_x,curr_y+1] = self.get_heuristic(curr_x,curr_y+1) + cost_g[curr_x,curr_y+1]
                    parents[curr_x,curr_y+1] = [curr_x,curr_y]
            # We're done mark cell as visited
            is_visited[curr_x, curr_y] = 1
       
        return path

    

    def run(self, rate: float = 1):

        while not rospy.is_shutdown():

            if self.publish:
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