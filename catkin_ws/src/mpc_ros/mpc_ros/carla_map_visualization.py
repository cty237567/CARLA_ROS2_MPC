#!/usr/bin/env python3

import numpy as np
import carla

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from carla_msgs.msg import CarlaWorldInfo

COLOR_ALUMINIUM_2 = ColorRGBA(r=186.0 / 255.0, g=189.0 / 255.0, b=182.0 / 255.0, a=1.0)
COLOR_SKY_BLUE_0 = ColorRGBA(r=114.0 / 255.0, g=159.0 / 255.0, b=207.0 / 255.0, a=1.0)
COLOR_CHAMELEON_0 = ColorRGBA(r=138.0 / 255.0, g=226.0 / 255.0, b=52.0 / 255.0, a=1.0)
COLOR_SCARLET_RED_0 = ColorRGBA(r=239.0 / 255.0, g=41.0 / 255.0, b=41.0 / 255.0, a=1.0)
COLOR_ORANGE_0 = ColorRGBA(r=252.0 / 255.0, g=175.0 / 255.0, b=62.0 / 255.0, a=1.0)


class CarlaMapVisualization(Node):
    def __init__(self):
        super().__init__('carla_map_visualization')
        self.world = None
        self.connect_to_carla()
        self.map = self.world.get_map()
        self.map_name = self.map.name
        self.get_logger().info(f"Map Visualization Node: Loading {self.map_name} map!")

        self.publisher = self.create_publisher(MarkerArray, '/carla/map_visualization', 1)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.id = 0
        self.marker_array = MarkerArray()
        self.draw_map()

    def connect_to_carla(self):
        self.get_logger().info("Waiting for CARLA world (topic: /carla/world_info)...")

        # ROS2 doesn't support wait_for_message natively, so we assume Carla is running
        host = self.get_parameter_or('host', '127.0.0.1')
        port = self.get_parameter_or('port', 2000)
        timeout = self.get_parameter_or('timeout', 10.0)

        self.get_logger().info(f"Connecting to CARLA at {host}:{port}...")
        carla_client = carla.Client(host, port)
        carla_client.set_timeout(timeout)

        try:
            self.world = carla_client.get_world()
        except RuntimeError as e:
            self.get_logger().error(f"Error while connecting to CARLA: {e}")
            raise e

        self.get_logger().info("Connected to CARLA.")

    def timer_callback(self):
        self.publisher.publish(self.marker_array)

    @staticmethod
    def lateral_shift(transform, shift):
        transform.rotation.yaw += 90
        return transform.location + shift * transform.get_forward_vector()

    def set_marker_id(self):
        self.id += 1
        return self.id - 1

    def add_arrow_line_marker(self, transform):
        arrow_marker = Marker()
        arrow_marker.type = Marker.LINE_LIST
        
        arrow_marker.header.frame_id = "map"
        arrow_marker.id = self.set_marker_id()
        arrow_marker.ns = "map_visualization"
        arrow_marker.color = ColorRGBA(r=0.8, g=0.8, b=0.8, a=1.0)
        arrow_marker.scale.x = 0.2
        arrow_marker.pose.orientation.w = 1.0

        transform.rotation.yaw += 180
        forward = transform.get_forward_vector()
        transform.rotation.yaw += 90
        right_dir = transform.get_forward_vector()
        end = transform.location
        start = end - 2.0 * forward
        right = start + 0.8 * forward + 0.4 * right_dir
        left = start + 0.8 * forward - 0.4 * right_dir

        for p in [start, end, start, left, start, right]:
            point = Point()
            point.x = p.x
            point.y = -p.y
            point.z = -2.0
            arrow_marker.points.append(point)

        self.marker_array.markers.append(arrow_marker)

    def add_line_strip_marker(self, color=None, points=None):
        marker = Marker()
        marker.id = self.set_marker_id()
        marker.type = Marker.LINE_STRIP
        marker.header.frame_id = "map"
        marker.ns = "map_visualization"
        marker.color = color or ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        marker.scale.x = 0.25
        marker.pose.orientation.w = 1.0

        if points is not None:
            for p in points:
                point = Point()
                point.x = p.x
                point.y = -p.y
                point.z = -2.0
                marker.points.append(point)

        self.marker_array.markers.append(marker)
        return marker

    def draw_map(self):
        precision = 0.1
        topology = self.map.get_topology()
        topology = [x[0] for x in topology]
        topology = sorted(topology, key=lambda w: w.transform.location.z)

        set_waypoints = []
        for waypoint in topology:
            waypoints = [waypoint]
            nxt = waypoint.next(precision)
            if len(nxt) > 0:
                nxt = nxt[0]
                while nxt.road_id == waypoint.road_id:
                    waypoints.append(nxt)
                    nxt = nxt.next(precision)
                    if len(nxt) > 0:
                        nxt = nxt[0]
                    else:
                        break
            set_waypoints.append(waypoints)

        for waypoints in set_waypoints:
            waypoint = waypoints[0]
            road_left_side = [self.lateral_shift(w.transform, -w.lane_width * 0.5) for w in waypoints]
            road_right_side = [self.lateral_shift(w.transform, w.lane_width * 0.5) for w in waypoints]

            if len(road_left_side) > 2:
                self.add_line_strip_marker(points=road_left_side)
            if len(road_right_side) > 2:
                self.add_line_strip_marker(points=road_right_side)

            if not waypoint.is_junction:
                for n, wp in enumerate(waypoints):
                    if ((n + 1) % 400) == 0:
                        self.add_arrow_line_marker(wp.transform)


def main(args=None):
    rclpy.init(args=args)
    viz = CarlaMapVisualization()
    try:
        rclpy.spin(viz)
    except KeyboardInterrupt:
        viz.get_logger().info('Shutting down Carla Map Visualization node.')
    viz.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()