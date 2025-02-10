#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

import tf2_geometry_msgs as tfg
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

qos_profile = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

class TranformPoints(Node):
    """Demonstrating some convertions and loading the map as an image"""
    def __init__(self):
        super().__init__('transform_point')

        # Basic ROS stuff
        timer_frequency = 1
        timer_period = 1/timer_frequency

        # Functionality variables
        self.marker_id = 0

        # For listening and loading the 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # For publishing the markers
        self.marker_pub = self.create_publisher(Marker, "/breadcrumbs", QoSReliabilityPolicy.BEST_EFFORT)

        # Create a timer, to do the main work.
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Create a PointStamped in the /base_link frame of the robot
        # The point is located 0.5m in from of the robot
        # "Stamped" means that the message type contains a Header
        point_in_robot_frame = PointStamped()
        point_in_robot_frame.header.frame_id = "/base_link"
        point_in_robot_frame.header.stamp = self.get_clock().now().to_msg()

        point_in_robot_frame.point.x = -0.5
        point_in_robot_frame.point.y = 0.
        point_in_robot_frame.point.z = 0. 

        # Now we look up the transform between the base_link and the map frames
        # and then we apply it to our PointStamped
        time_now = rclpy.time.Time()
        timeout = Duration(seconds=0.1)
        try:
            # An example of how you can get a transform from /base_link frame to the /map frame
            # as it is at time_now, wait for timeout for it to become available
            trans = self.tf_buffer.lookup_transform("map", "base_link", time_now, timeout)
            self.get_logger().info(f"Looks like the transform is available.")

            # Now we apply the transform to transform the point_in_robot_frame to the map frame
            # The header in the result will be copied from the Header of the transform
            point_in_map_frame = tfg.do_transform_point(point_in_robot_frame, trans)
            self.get_logger().info(f"We transformed a PointStamped!")

            # If the transformation exists, create a marker from the point, in order to visualize it in Rviz
            marker_in_map_frame = self.create_marker(point_in_map_frame, self.marker_id)

            # Publish the marker
            self.marker_pub.publish(marker_in_map_frame)
            self.get_logger().info(f"The marker has been published to /breadcrumbs. You are able to visualize it in Rviz")

            # Increase the marker_id, so we dont overwrite the same marker.
            self.marker_id += 1

        except TransformException as te:
            self.get_logger().info(f"Cound not get the transform: {te}")



    def create_marker(self, point_stamped, marker_id, lifetime=30.0):
        """You can see the description of the Marker message here: https://docs.ros2.org/galactic/api/visualization_msgs/msg/Marker.html"""
        marker = Marker()

        marker.header = point_stamped.header

        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.id = marker_id

        # Set the scale of the marker
        scale = 0.1
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        # Set the color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position.x = point_stamped.point.x
        marker.pose.position.y = point_stamped.point.y
        marker.pose.position.z = point_stamped.point.z

        marker.lifetime = Duration(seconds=lifetime).to_msg()

        return marker

def main():

    rclpy.init(args=None)
    node = TranformPoints()
    
    rclpy.spin(node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()