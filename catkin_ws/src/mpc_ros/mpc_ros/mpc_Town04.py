#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from carla_msgs.msg import CarlaEgoVehicleControl
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

import numpy as np
from math import atan2, sin, cos, pi, hypot
from mpc_ros.mpc.mpc_path_tracking import mpc_path_tracking
import csv
import os
from datetime import datetime

scaling = 10
GOAL_REACHED_THRESHOLD = 2.0  # Stop within 2 meters of the final waypoint

class MPCCore(Node):
    def __init__(self):
        super().__init__('mpc_node')
        # Declare parameters
        self.declare_parameters('', [
            ('receding', 8), ('max_speed', 8.0), ('ref_speed', 4.0),
            ('max_acce', 1.0), ('max_acce_steer', 0.02), ('sample_time', 0.2),
            ('max_steer', pi/6), ('wheelbase', 2.87),
            ('shape', [4.69, 1.85, 2.87, 1.75]),  # Store shape info
            ('cs', [1, 1, 1]), ('cu', 1.0), ('cst', [1, 1, 1]), ('cut', 1.0),
            ('odom_topic', '/carla/agent_0/odometry'),
            ('ctl_topic', '/carla/agent_0/vehicle_control_cmd')
        ])
        # Load parameters
        hz = self.get_parameter('receding').value
        spd = self.get_parameter('max_speed').value
        rspd = self.get_parameter('ref_speed').value
        amax = self.get_parameter('max_acce').value
        smax = self.get_parameter('max_acce_steer').value
        Ts = self.get_parameter('sample_time').value
        sdeg = self.get_parameter('max_steer').value
        wb = self.get_parameter('wheelbase').value
        self.shape = self.get_parameter('shape').value  # Store shape parameter
        cs = np.diag(self.get_parameter('cs').value)
        cu = self.get_parameter('cu').value
        cst = np.diag(self.get_parameter('cst').value)
        cut = self.get_parameter('cut').value

        # MPC controller
        self.mpc = mpc_path_tracking(
            receding=hz, max_speed=spd, ref_speed=rspd,
            max_acce=amax, max_acce_steer=smax,
            sample_time=Ts, max_steer=sdeg, wheelbase=wb,
            cs=cs, cu=cu, cst=cst, cut=cut)

        # State & outputs
        self.state = np.zeros((3, 1))
        self.output = CarlaEgoVehicleControl()
        self.ref_path = []
        self.current_wp_index = 0  # Track progress along the path
        self.goal_reached = False

        # Publishers
        ctl_topic = self.get_parameter('ctl_topic').value
        self.pub_ctl = self.create_publisher(CarlaEgoVehicleControl, ctl_topic, 10)
        self.pub_ref = self.create_publisher(Path, 'dubin_path', 10)
        self.pub_opt = self.create_publisher(Path, 'opt_path', 10)
        self.pub_mrk = self.create_publisher(MarkerArray, 'car_marker', 10)

        # Subscriptions
        odom_t = self.get_parameter('odom_topic').value
        self.create_subscription(Odometry, odom_t, self.cb_odom, 10)
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, '/carla/agent_0/waypoints',
                                 self.cb_waypoints, qos)

        # Timer
        self.create_timer(Ts, self.control_loop)
        self.get_logger().info("MPC Node initialized and waiting for waypoints.")

        # Logging setup
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = os.path.expanduser("~/mpc_logs")
        os.makedirs(log_dir, exist_ok=True)
        self.vehicle_log_file = os.path.join(log_dir, f"vehicle_data_{timestamp}.csv")  ## This is saved in home directory as mpc_logs
        self.control_log_file = os.path.join(log_dir, f"control_data_{timestamp}.csv")
        
        # Initialize CSV files with headers
        with open(self.vehicle_log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Time', 'X', 'Y', 'Yaw', 'Closest_WP_Index'])
        with open(self.control_log_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Time', 'Throttle', 'Steer', 'Brake'])

    def cb_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = atan2(2*(q.w*q.z+q.x*q.y),
                    1-2*(q.y*q.y+q.z*q.z))
        offset = self.shape[2]/2
        self.state[0] = x - offset*cos(yaw)
        self.state[1] = y - offset*sin(yaw)
        self.state[2] = yaw
        
        # Log vehicle state
        current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        with open(self.vehicle_log_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([current_time, self.state[0], self.state[1], self.state[2], self.current_wp_index])

    def cb_waypoints(self, msg: Path):
        self.ref_path = []
        for ps in msg.poses:
            x = ps.pose.position.x
            y = ps.pose.position.y
            q = ps.pose.orientation
            yaw = atan2(2*(q.w*q.z+q.x*q.y),
                        1-2*(q.y*q.y+q.z*q.z))
            self.ref_path.append(np.array([[x],[y],[yaw]]))
        # Publish reference path for RViz
        self.pub_ref.publish(self.make_path(self.ref_path))
        self.get_logger().info(f"Loaded {len(self.ref_path)} waypoints.")
        self.current_wp_index = 0  # Reset progress on new path
        self.goal_reached = False

    def control_loop(self):
        if not self.ref_path or self.goal_reached:
            # If goal reached, ensure vehicle is stopped
            if self.goal_reached:
                self.output.throttle = 0.0
                self.output.steer = 0.0
                self.output.brake = 1.0  # Apply brake
                self.pub_ctl.publish(self.output)
            return

        # Find closest point on path ahead of current index
        min_dist = float('inf')
        closest_idx = self.current_wp_index
        car_xy = self.state[0:2, 0]

        # Search from current index onwards
        for i in range(self.current_wp_index, len(self.ref_path)):
            pt_xy = self.ref_path[i][0:2, 0]
            dist = hypot(pt_xy[0] - car_xy[0], pt_xy[1] - car_xy[1])
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
            # Optimization: stop searching if distance starts increasing significantly
            elif dist > min_dist + 5.0:
                break
        self.current_wp_index = closest_idx

        # Check if goal is reached (close to the last waypoint)
        if self.current_wp_index >= len(self.ref_path) - 1 and min_dist < GOAL_REACHED_THRESHOLD:
            self.get_logger().info("Goal reached! Stopping vehicle.")
            self.goal_reached = True
            self.output.throttle = 0.0
            self.output.steer = 0.0
            self.output.brake = 1.0
            self.pub_ctl.publish(self.output)
            return

        # Prepare reference for MPC horizon
        hz = self.get_parameter('receding').value
        # Ensure we don't go past the end of the path
        end_idx = min(self.current_wp_index + hz + 1, len(self.ref_path))
        ref_horizon = self.ref_path[self.current_wp_index:end_idx]

        # If horizon is too short (near end of path), pad with last point
        while len(ref_horizon) < hz + 1:
            ref_horizon.append(self.ref_path[-1])

        # Solve MPC
        u, info, flag, xopt = self.mpc.controller(
            self.state, ref_horizon, iter_num=5)  # Pass the horizon slice

        if flag or u is None:
            self.get_logger().warn("MPC solver failed, applying zero control")
            self.output.throttle = 0.0
            self.output.steer = 0.0
            self.output.brake = 0.5  # Gentle brake on failure
        else:
            self.output.throttle = float(np.clip(u[0, 0]/scaling, 0.0, 1.0))
            self.output.steer = float(np.clip(-u[1, 0], -1.0, 1.0))
            self.output.brake = 0.0  # Release brake if MPC solved

        # Publish control
        self.pub_ctl.publish(self.output)

        # Log control inputs
        current_time = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        with open(self.control_log_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([current_time, self.output.throttle, self.output.steer, self.output.brake])

        # Publish optimized path ONLY if it exists
        if xopt is not None:
            self.pub_opt.publish(self.make_path(xopt))
        else:
            # Don't publish opt_path if solver failed to produce it
            pass

        # Publish reference path (always available)
        self.pub_ref.publish(self.make_path(self.ref_path))

    def make_path(self, pts):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for pt in pts:
            ps = PoseStamped()
            if pt.ndim == 2 and pt.shape[0] >= 3:  # [[x],[y],[yaw],...]
                x, y, yaw = pt[0, 0], pt[1, 0], pt[2, 0]
            elif pt.ndim == 1 and len(pt) >= 3:  # [x, y, yaw,...]
                x, y, yaw = pt[0], pt[1], pt[2]
            else:
                self.get_logger().warn(f"Skipping invalid point shape: {pt.shape}")
                continue
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation = Quaternion(w=cos(yaw/2), z=sin(yaw/2))
            path.poses.append(ps)
        return path

def main(args=None):
    rclpy.init(args=args)
    node = MPCCore()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()
