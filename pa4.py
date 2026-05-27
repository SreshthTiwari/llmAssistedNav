#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Twist, Pose, Quaternion
from tf2_ros import Buffer, TransformListener, TransformException


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class OccupancyGridMapper(Node):
    def __init__(self):
        super().__init__('occupancy_grid_mapper')

        # Topics / frames
        self.declare_parameter('scan_topic', '/base_scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_frame', 'rosbot1/odom')
        self.declare_parameter('base_frame', 'rosbot1/base_link')

        # Map settings
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('initial_width', 200)
        self.declare_parameter('initial_height', 200)

        # Mapping settings
        self.declare_parameter('scan_skip', 1)
        self.declare_parameter('beam_stride', 2)      # process every Nth beam
        self.declare_parameter('l_occ', 1.2)
        self.declare_parameter('l_free', -0.8)
        self.declare_parameter('l_min', -5.0)
        self.declare_parameter('l_max', 5.0)
        self.declare_parameter('max_range_fallback', 10.0)

        # Motion settings
        self.declare_parameter('auto_move', True)
        self.declare_parameter('forward_speed', 0.15)
        self.declare_parameter('turn_speed', 0.55)
        self.declare_parameter('front_clear_dist', 0.9)
        self.declare_parameter('turn_duration_sec', 1.3)
        self.declare_parameter('move_duration_sec', 2.2)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.resolution = float(self.get_parameter('resolution').value)
        self.width = int(self.get_parameter('initial_width').value)
        self.height = int(self.get_parameter('initial_height').value)

        self.scan_skip = int(self.get_parameter('scan_skip').value)
        self.beam_stride = int(self.get_parameter('beam_stride').value)
        self.l_occ = float(self.get_parameter('l_occ').value)
        self.l_free = float(self.get_parameter('l_free').value)
        self.l_min = float(self.get_parameter('l_min').value)
        self.l_max = float(self.get_parameter('l_max').value)
        self.max_range_fallback = float(self.get_parameter('max_range_fallback').value)

        self.auto_move = bool(self.get_parameter('auto_move').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.front_clear_dist = float(self.get_parameter('front_clear_dist').value)
        self.turn_duration_sec = float(self.get_parameter('turn_duration_sec').value)
        self.move_duration_sec = float(self.get_parameter('move_duration_sec').value)

        # Map origin in odom frame
        self.origin_x = -(self.width * self.resolution) / 2.0
        self.origin_y = -(self.height * self.resolution) / 2.0

        # Log-odds storage
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self.observed = np.zeros((self.height, self.width), dtype=np.bool_)

        self.scan_count = 0
        self.last_front_scan = float('inf')
        self.motion_state = 'turn'
        self.motion_start = self.get_clock().now()

        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_timer = self.create_timer(0.5, self.publish_map)
        self.drive_timer = self.create_timer(0.1, self.drive_robot)

        self.get_logger().info("OccupancyGridMapper started.")

    # ---------------------------
    # Coordinate helpers
    # ---------------------------
    def world_to_map(self, x, y):
        mx = int((x - self.origin_x) / self.resolution)
        my = int((y - self.origin_y) / self.resolution)
        return mx, my

    def in_bounds(self, mx, my):
        return 0 <= mx < self.width and 0 <= my < self.height

    def grow_map_if_needed(self, mx, my):
        grow_left = max(0, -mx)
        grow_bottom = max(0, -my)
        grow_right = max(0, mx - self.width + 1)
        grow_top = max(0, my - self.height + 1)

        if grow_left == 0 and grow_right == 0 and grow_bottom == 0 and grow_top == 0:
            return

        new_w = self.width + grow_left + grow_right
        new_h = self.height + grow_bottom + grow_top

        new_log = np.zeros((new_h, new_w), dtype=np.float32)
        new_obs = np.zeros((new_h, new_w), dtype=np.bool_)

        x_off = grow_left
        y_off = grow_bottom
        new_log[y_off:y_off + self.height, x_off:x_off + self.width] = self.log_odds
        new_obs[y_off:y_off + self.height, x_off:x_off + self.width] = self.observed

        self.origin_x -= grow_left * self.resolution
        self.origin_y -= grow_bottom * self.resolution

        self.width = new_w
        self.height = new_h
        self.log_odds = new_log
        self.observed = new_obs

        self.get_logger().warn(f"Map grew to {self.width}x{self.height}")

    # ---------------------------
    # Bresenham line algorithm
    # ---------------------------
    def bresenham(self, x0, y0, x1, y1):
        cells = []

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0
        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        return cells

    # ---------------------------
    # Log-odds update
    # ---------------------------
    def update_cell(self, mx, my, delta):
        self.grow_map_if_needed(mx, my)
        if self.in_bounds(mx, my):
            self.log_odds[my, mx] = np.clip(self.log_odds[my, mx] + delta, self.l_min, self.l_max)
            self.observed[my, mx] = True

    def logodds_to_occupancy(self, l):
        p = 1.0 / (1.0 + math.exp(-l))
        if p > 0.55:
            return 100
        elif p < 0.45:
            return 0
        return -1

    # ---------------------------
    # Scan callback
    # ---------------------------
    def scan_callback(self, msg):
        self.scan_count += 1
        if self.scan_count % self.scan_skip != 0:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                # msg.header.stamp,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2)
            )
        except TransformException as ex:
            self.get_logger().warn(f"TF lookup failed: {ex}")
            return

        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        yaw = yaw_from_quaternion(tf.transform.rotation)

        robot_mx, robot_my = self.world_to_map(rx, ry)
        self.grow_map_if_needed(robot_mx, robot_my)

        # Use middle beam to estimate front range for motion
        if len(msg.ranges) > 0:
            mid = len(msg.ranges) // 2
            front = msg.ranges[mid]
            if math.isfinite(front):
                self.last_front_scan = front

        angle = msg.angle_min
        for i in range(0, len(msg.ranges), self.beam_stride):
            rng = msg.ranges[i]

            if math.isnan(rng) or math.isinf(rng) or rng < msg.range_min:
                angle += self.beam_stride * msg.angle_increment
                continue

            max_r = msg.range_max if msg.range_max > 0.0 else self.max_range_fallback
            dist = min(rng, max_r)

            beam_angle = yaw + angle
            end_x = rx + dist * math.cos(beam_angle)
            end_y = ry + dist * math.sin(beam_angle)

            end_mx, end_my = self.world_to_map(end_x, end_y)
            self.grow_map_if_needed(end_mx, end_my)

            cells = self.bresenham(robot_mx, robot_my, end_mx, end_my)

            # Free cells along ray
            for cx, cy in cells[:-1]:
                self.update_cell(cx, cy, self.l_free)

            # Occupied cell only if real hit, not max range
            if rng < max_r:
                hx, hy = cells[-1]
                self.update_cell(hx, hy, self.l_occ)

            angle += self.beam_stride * msg.angle_increment

    # ---------------------------
    # Simple autonomous motion
    # ---------------------------
    def drive_robot(self):
        if not self.auto_move:
            self.cmd_pub.publish(Twist())
            return

        cmd = Twist()

        # Hard safety stop
        if self.last_front_scan < 0.7:
            cmd.angular.z = self.turn_speed
            cmd.linear.x = 0.0
            self.motion_state = 'turn'
            self.motion_start = self.get_clock().now()
            self.cmd_pub.publish(cmd)
            return

        now = self.get_clock().now()
        elapsed = (now - self.motion_start).nanoseconds * 1e-9

        if self.motion_state == 'turn':
            cmd.angular.z = self.turn_speed
            if elapsed >= self.turn_duration_sec:
                self.motion_state = 'forward'
                self.motion_start = now

        elif self.motion_state == 'forward':
            if self.last_front_scan > self.front_clear_dist:
                cmd.linear.x = self.forward_speed
            else:
                self.motion_state = 'turn'
                self.motion_start = now

            if elapsed >= self.move_duration_sec:
                self.motion_state = 'turn'
                self.motion_start = now

        self.cmd_pub.publish(cmd)

    # ---------------------------
    # Publish occupancy grid
    # ---------------------------
    def publish_map(self):
        grid = OccupancyGrid()
        grid.header.frame_id = self.odom_frame
        grid.header.stamp = self.get_clock().now().to_msg()

        meta = MapMetaData()
        meta.resolution = float(self.resolution)
        meta.width = int(self.width)
        meta.height = int(self.height)

        origin = Pose()
        origin.position.x = float(self.origin_x)
        origin.position.y = float(self.origin_y)
        origin.position.z = 0.0
        origin.orientation = quat_from_yaw(0.0)
        meta.origin = origin

        grid.info = meta

        data = np.empty((self.height, self.width), dtype=np.int8)
        for y in range(self.height):
            for x in range(self.width):
                if not self.observed[y, x]:
                    data[y, x] = -1
                else:
                    data[y, x] = self.logodds_to_occupancy(self.log_odds[y, x])

        grid.data = data.flatten(order='C').tolist()
        self.map_pub.publish(grid)


def main():
    rclpy.init()
    node = OccupancyGridMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()