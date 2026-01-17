#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
import csv
import os

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit_real')

        # ========== PARAMETERS ==========
        self.declare_parameter("waypoint_path", "/sim_ws/install/waypoint/share/waypoint/f1tenth_waypoint_generator/racelines/f1tenth_waypoint.csv")
        self.declare_parameter("lookahead_dist", 1.0)
        self.declare_parameter("wheelbase", 0.33)
        self.declare_parameter("max_speed", 1.0)

        self.csv_path = self.get_parameter("waypoint_path").value
        self.L = self.get_parameter("lookahead_dist").value
        self.wheelbase = self.get_parameter("wheelbase").value
        self.max_speed = self.get_parameter("max_speed").value

        # ========== LOAD WAYPOINT ==========
        self.waypoints = self.load_waypoints(self.csv_path)
        self.last_idx = 0

        # ========== PUB / SUB ==========
        self.create_subscription(
            Odometry,
            "/odom",            # 🔥 ODOM TỪ XE THẬT
            self.odom_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            AckermannDriveStamped,
            "/drive",     # 🔥 COMMAND TỚI XE
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/pure_pursuit/markers",
            10
        )

        self.publish_static_path()
        self.get_logger().info("Pure Pursuit (REAL VEHICLE) READY")

    # ================= CORE =================

    def load_waypoints(self, path):
        pts = []
        if not os.path.exists(path):
            self.get_logger().error("Waypoint file not found!")
            return np.array([])
        with open(path) as f:
            reader = csv.reader(f)
            next(reader)
            for r in reader:
                pts.append([float(r[0]), float(r[1])])
        return np.array(pts)

    def odom_callback(self, msg):
        if len(self.waypoints) == 0:
            return

        # --- Pose ---
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )

        target = self.get_target_point(x, y)
        steering = self.calculate_steering(target, x, y, yaw)

        self.publish_cmd(steering)
        self.publish_markers(target, x, y)

    # ================= PURE PURSUIT =================

    def get_target_point(self, x, y):
        n = len(self.waypoints)
        indices = [(self.last_idx + i) % n for i in range(50)]

        pts = self.waypoints[indices]
        dists = np.linalg.norm(pts - np.array([x, y]), axis=1)

        nearest = indices[np.argmin(dists)]
        self.last_idx = nearest

        idx = nearest
        while True:
            idx = (idx + 1) % n
            if math.dist([x, y], self.waypoints[idx]) >= self.L:
                return self.waypoints[idx]
            if idx == nearest:
                return self.waypoints[idx]

    def calculate_steering(self, target, x, y, yaw):
        dx = target[0] - x
        dy = target[1] - y

        y_local = dx * math.sin(-yaw) + dy * math.cos(-yaw)
        ld = math.dist([x, y], target)

        return math.atan2(2 * self.wheelbase * y_local, ld**2)

    # ================= OUTPUT =================

    def publish_cmd(self, angle):
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = float(angle)

        speed = self.max_speed
        if abs(angle) > 0.35:
            speed *= 0.5

        msg.drive.speed = speed
        self.cmd_pub.publish(msg)

    # ================= VISUAL =================

    def publish_static_path(self):
        if len(self.waypoints) == 0:
            return
        m = Marker()
        m.header.frame_id = "odom"
        m.type = Marker.SPHERE_LIST
        m.scale.x = m.scale.y = m.scale.z = 0.1
        m.color.a = 1.0
        m.color.r = 1.0
        for p in self.waypoints:
            pt = Point()
            pt.x, pt.y = p
            m.points.append(pt)
        arr = MarkerArray()
        arr.markers.append(m)
        self.marker_pub.publish(arr)

    def publish_markers(self, target, x, y):
        arr = MarkerArray()
        arr.markers.append(self.create_marker(0, target[0], target[1], 0,1,0))
        arr.markers.append(self.create_marker(1, x, y, 0,0,1))
        self.marker_pub.publish(arr)

    def create_marker(self, id, x, y, r, g, b):
        m = Marker()
        m.header.frame_id = "odom"
        m.id = id
        m.type = Marker.SPHERE
        m.scale.x = m.scale.y = m.scale.z = 0.3
        m.color.a = 1.0
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.pose.position.x = x
        m.pose.position.y = y
        return m


def main():
    rclpy.init()
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
