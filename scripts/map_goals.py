#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Quaternion, PoseStamped
from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler
# from tf_transformations import quaternion_from_euler

import tf_transformations

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data

import cv2
import numpy as np

qos_profile = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

class MapGoals(Node):
    """Demonstrating some convertions and loading the map as an image"""
    def __init__(self):
        super().__init__('map_goals')

        # Basic ROS stuff
        timer_frequency = 2
        map_topic = "/map"
        timer_period = 1/timer_frequency

        # Functional variables
        self.pending_goal = False
        self.result_future = None
        self.currently_navigating = False
        self.clicked_x = None
        self.clicked_y = None
        self.ros_occupancy_grid = None
        self.map_np = None
        self.map_data = {"map_load_time":None,
                         "resolution":None,
                         "width":None,
                         "height":None,
                         "origin":None} # origin will be in the format [x,y,theta]

        # Subscribe to map, and create an action client for sending goals
        self.occupancy_grid_sub = self.create_subscription(OccupancyGrid, map_topic, self.map_callback, qos_profile)
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Create a timer, to do the main work.
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f"Node has been initialized! Will perform the tasks.")

        cv2.namedWindow("ROS2 map", cv2.WINDOW_NORMAL)

    def click_event(self, event, x, y, flags, params): 
    
        self.clicked_x = x
        self.clicked_y = y
        # checking for left mouse clicks 
        if event == cv2.EVENT_LBUTTONDOWN: 
            self.get_logger().info(f"Clicked a new point: {x}, {y}.")
            if not self.currently_navigating:
                self.get_logger().info(f"Will generate and send a new goal! Eventually.")
                self.clicked_x = x
                self.clicked_y = y
                self.pending_goal = True
            else:
                self.get_logger().info(f"Robot is being navigated, goal rejected!")

    def timer_callback(self):
        if self.map_np is None:
            self.get_logger().info(f"Waiting for a new map to be loaded!")
            return
        
        # if not self.map_np is None and not self.clicked_x is None:
        #     world_x, world_y = self.map_pixel_to_world(self.clicked_x, self.clicked_y)
        #     world_orientation = 0.            
        #     x, y = self.world_to_map_pixel(world_x, world_y)
        #     # Put a one pixel dot in the map image, to verify conversion
        #     self.map_np[int(y-1)][int(x-1)] = 0
        
        # If the robot is not currently navigating to a goal, and there is a goal pending
        if not self.currently_navigating and self.pending_goal:
            world_x, world_y = self.map_pixel_to_world(self.clicked_x, self.clicked_y)
            world_orientation = 0.
            goal_pose = self.generate_goal_message(world_x, world_y, world_orientation)
            self.go_to_pose(goal_pose)

            # Not needed, just checking the coordinate conversions


        cv2.imshow("ROS2 map", self.map_np)
        cv2.setMouseCallback("ROS2 map", self.click_event)
        key = cv2.waitKey(1)

    def generate_goal_message(self, x, y, theta=0.2):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation = self.yaw_to_quaternion(theta)

        return goal_pose

    def map_pixel_to_world(self, x, y, theta=0):
        ### Convert a pixel in an numpy image, to a real world location
        ### Works only for theta=0
        assert not self.map_data["resolution"] is None

        # Apply resolution, change of origin, and translation
        # 
        world_x = x*self.map_data["resolution"] + self.map_data["origin"][0]
        world_y = (self.map_data["height"]-y)*self.map_data["resolution"] + self.map_data["origin"][1]

        # Apply rotation
        return world_x, world_y

    def world_to_map_pixel(self, world_x, world_y, world_theta=0.2):
        ### Convert a real world location to a pixel in a numpy image
        ### Works only for theta=0
        assert self.map_data["resolution"] is not None

        # Apply resolution, change of origin, and translation
        # x is the first coordinate, which in opencv and numpy that is the matrix row - vertical
        # vertical
        x = int((world_x - self.map_data["origin"][0])/self.map_data["resolution"])
        y = int(self.map_data["height"] - (world_y - self.map_data["origin"][1])/self.map_data["resolution"] )
        
        # Apply rotation
        return x, y

    def map_callback(self, msg):
        self.get_logger().info(f"Read a new Map (Occupancy grid) from the topic.")
        # reshape the message vector back into a map
        self.map_np = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        # fix the direction of Y (origin at top for OpenCV, origin at bottom for ROS2)
        self.map_np = np.flipud(self.map_np)
        # change the colors so they match with the .pgm image
        self.map_np[self.map_np==0] = 127
        self.map_np[self.map_np==100] = 0
        # load the map parameters
        self.map_data["map_load_time"]=msg.info.map_load_time
        self.map_data["resolution"]=msg.info.resolution
        self.map_data["width"]=msg.info.width
        self.map_data["height"]=msg.info.height
        quat_list = [msg.info.origin.orientation.x,
                     msg.info.origin.orientation.y,
                     msg.info.origin.orientation.z,
                     msg.info.origin.orientation.w]
        self.map_data["origin"]=[msg.info.origin.position.x,
                                 msg.info.origin.position.y,
                                 tf_transformations.euler_from_quaternion(quat_list)[-1]]
        #self.get_logger().info(f"Read a new Map (Occupancy grid) from the topic.")

    def go_to_pose(self, pose):
        """Send a `NavToPose` action request."""
        self.currently_navigating = True
        self.pending_goal = False

        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = ""

        self.get_logger().info('Attempting to navigate to goal: ' + str(pose.pose.position.x) + ' ' + str(pose.pose.position.y) + '...')
        self.send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        
        # Call this function when the Action Server accepts or rejects a goal
        self.send_goal_future.add_done_callback(self.goal_accepted_callback)

    def goal_accepted_callback(self, future):
        """Here we do something, depending on whether the ActionServer ACCEPTED our goal"""
        goal_handle = future.result()

        # If the goal was accepted
        if goal_handle.accepted: 
            self.get_logger().info('Goal was accepted!')
            # Set the correct flags
            self.currently_navigating = True
            self.pending_goal = False
            # Set the Future object, and callback function for reading the result of the action
            self.result_future = goal_handle.get_result_async()
            self.result_future.add_done_callback(self.get_result_callback)
        elif not goal_handle.accepted:
            self.get_logger().error('Goal was rejected!')

    def get_result_callback(self, future):
        """Here we do something, depending on whether the ActionServer has REACHED our goal"""
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Goal failed with status code: {status}')
        else:
            self.get_logger().info(f'Goal reached (according to Nav2).')

        self.currently_navigating = False
        self.pending_goal = False

    def yaw_to_quaternion(self, angle_z = 0.):
        quat_tf = quaternion_from_euler(0, 0, angle_z)

        # Convert a list to geometry_msgs.msg.Quaternion
        quat_msg = Quaternion(x=quat_tf[0], y=quat_tf[1], z=quat_tf[2], w=quat_tf[3]) # for tf_turtle
        # quat_msg = Quaternion(x=quat_tf[1], y=quat_tf[2], z=quat_tf[3], w=quat_tf[0]) # for tf transforms


        # tf returns the quaternion with the ordering x, y, z, w and transforms3d returns w, x, y, z.

        return quat_msg
    
    def get_rotation_matrix(self, theta):
        c = np.cos(theta)
        s = np.sin(theta)
        rot = np.array([[c, -s],
                        [s , c]])
        return rot

def main():
    #print('Navigate to a goal by clicking a pixel.')

    rclpy.init(args=None)
    node = MapGoals()
    
    rclpy.spin(node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()