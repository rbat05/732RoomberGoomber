#!/usr/bin/env python3
"""
Autonomous Cube Finder — COMPSYS732
Phase 2 autonomous search node.

States: SEARCHING → REPORTING → RETURNING → DONE

To run:
    ~/ros2_venv/bin/python3 -m tb4_sensor_reader.autonomous_search
"""

import rclpy, cv2, math, os, time
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, CompressedImage
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

# ── Robot namespace ────────────────────────────────────────────────────────────
NAMESPACE = '/T24'

# ── Motion parameters ──────────────────────────────────────────────────────────
FORWARD_SPEED    = 0.15      # m/s — waypoint navigation
TURN_SPEED       = 0.4       # rad/s — waypoint navigation turns
AVOID_TURN_SPEED = 0.5       # rad/s — obstacle avoidance turns
RETURN_SPEED     = 0.15      # m/s — return to start

# ── Tolerances ─────────────────────────────────────────────────────────────────
WAYPOINT_TOLERANCE  = 0.15   # m — close enough to waypoint to begin camera scan
HEADING_TOLERANCE   = 0.08   # rad — close enough to target heading to drive
RETURN_TOLERANCE    = 0.15   # m — close enough to origin to stop

# ── Obstacle avoidance ─────────────────────────────────────────────────────────
AVOID_DISTANCE   = 0.20      # m — obstacle trigger distance
FRONT_ARC_DEG    = 60        # degrees — front detection arc width
LIDAR_OFFSET_DEG = -90       # degrees — LiDAR mounting offset
LIDAR_OFFSET_RAD = math.radians(LIDAR_OFFSET_DEG)

# ── Time limit ─────────────────────────────────────────────────────────────────
TIME_LIMIT_S = 420.0         # 7 minutes — force RETURNING if cube not found

# ── Red cube detection ─────────────────────────────────────────────────────────
RED_LOW1   = np.array([0,   120, 70])
RED_HIGH1  = np.array([10,  255, 255])
RED_LOW2   = np.array([170, 120, 70])
RED_HIGH2  = np.array([180, 255, 255])
MIN_PIXELS = 5000

# ── Snapshot output path ───────────────────────────────────────────────────────
SNAPSHOT_PATH = os.path.expanduser('~/Desktop/detection_snapshot.jpg')

# ── LiDAR cube distance estimation ────────────────────────────────────────────
CUBE_FORWARD_ARC_DEG = 15    # degrees either side of forward for distance estimate
LIDAR_FALLBACK_DIST  = 0.30  # m — used if forward arc returns no valid ranges

# ── Cylinder waypoints ─────────────────────────────────────────────────────────
# These are (x, y) positions in the odometry frame recorded from the Phase 1
# SLAM map. Override manually if feature extraction is unavailable.
#
# Feature extraction is attempted first at startup from PGM_MAP_PATH.
# If it fails or PGM_MAP_PATH is None, MANUAL_WAYPOINTS is used directly.
#
PGM_MAP_PATH = os.path.expanduser('~/Desktop/test_map.pgm')   # set None to skip
PGM_MAP_YAML = os.path.expanduser('~/Desktop/test_map.yaml')  # for resolution/origin

MANUAL_WAYPOINTS = [
    (-2.3, -2.3),   # cylinder 1 — update from Rviz
    # (1.5, 0.5),   # cylinder 2 — update from Rviz
    # (1.5, 1.0),   # cylinder 3 — update from Rviz
]


# ── PGM feature extraction ─────────────────────────────────────────────────────

def extract_cylinder_waypoints(pgm_path, yaml_path):
    """
    Attempt to extract cylinder centre positions from a saved SLAM PGM map.
    Returns a list of (x, y) world-frame coordinates, or None on failure.

    Strategy: threshold the map to isolate occupied cells, find contours,
    filter by circularity and size to isolate cylinders, convert pixel
    centres to world coordinates using map resolution and origin.
    """
    try:
        import yaml

        # ── Load map metadata ────────────────────────────────────────────────
        with open(yaml_path, 'r') as f:
            meta = yaml.safe_load(f)
        resolution = meta['resolution']             # metres per pixel
        origin     = meta['origin']                 # [x, y, yaw] of bottom-left pixel

        # ── Load PGM ────────────────────────────────────────────────────────
        img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            return None

        # Occupied cells are dark (0), free cells are light (205 or 254)
        # Threshold: keep only occupied cells
        _, binary = cv2.threshold(img, 50, 255, cv2.THRESH_BINARY_INV)

        # ── Find contours ────────────────────────────────────────────────────
        contours, _ = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        waypoints = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 5 or area > 80:      # 3×3 to ~4×4 pixel blobs
                continue

            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue

            # Circularity: 1.0 = perfect circle
            # Lowered threshold — pixel-quantised blobs are blocky, not smooth circles
            circularity = 4 * math.pi * area / (perimeter ** 2)
            if circularity < 0.4:          # reject walls and linear features
                continue

            # Pixel centre of contour
            M = cv2.moments(cnt)
            if M['m00'] == 0:
                continue
            px = M['m10'] / M['m00']
            py = M['m01'] / M['m00']

            # Convert pixel → world coordinates
            # PGM origin is bottom-left; image row 0 is top → flip y
            world_x = origin[0] + px * resolution
            world_y = origin[1] + (img.shape[0] - py) * resolution
            waypoints.append((round(world_x, 3), round(world_y, 3)))

        if len(waypoints) == 0:
            return None

        # ── Save debug image with detected cylinder markers ──────────────────
        try:
            SCALE = 4   # upscale factor so small blobs are visible in debug image
            h, w = binary.shape
            debug_img = cv2.cvtColor(
                cv2.resize(binary, (w * SCALE, h * SCALE), interpolation=cv2.INTER_NEAREST),
                cv2.COLOR_GRAY2BGR)
            for (wx, wy) in waypoints:
                px_draw = int((wx - origin[0]) / resolution) * SCALE
                py_draw = int(img.shape[0] - (wy - origin[1]) / resolution) * SCALE
                cv2.circle(debug_img, (px_draw, py_draw), 10, (0, 0, 255), 2)  # red ring
                cv2.circle(debug_img, (px_draw, py_draw),  3, (0, 0, 255), -1) # red dot
                cv2.putText(
                    debug_img,
                    f'({wx:.2f},{wy:.2f})',
                    (px_draw + 12, py_draw),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 1)
            debug_path = os.path.splitext(pgm_path)[0] + '_features.jpg'
            cv2.imwrite(debug_path, debug_img)
        except Exception as e:
            pass

        if len(waypoints) == 0:
            return None

        return waypoints

    except Exception as e:
        return None


# ── Main node ──────────────────────────────────────────────────────────────────

class AutonomousSearch(Node):

    def __init__(self):
        super().__init__('autonomous_search')
        self.bridge = CvBridge()

        # ── Publishers ───────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(
            Twist, f'{NAMESPACE}/cmd_vel', 10)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(
            LaserScan, f'{NAMESPACE}/scan',
            self.scan_callback, 10)
        self.create_subscription(
            CompressedImage,
            f'{NAMESPACE}/oakd/rgb/image_raw/compressed',
            self.image_callback, 10)
        self.create_subscription(
            Odometry, f'{NAMESPACE}/odom',
            self.odom_callback, 10)

        # ── Sensor state ─────────────────────────────────────────────────────
        self.current_x      = 0.0
        self.current_y      = 0.0
        self.current_yaw    = 0.0    # radians
        self.nearest_front  = float('inf')
        self.nearest_left   = float('inf')
        self.nearest_right  = float('inf')
        self.latest_scan    = None   # full LaserScan msg for cube distance estimate
        self.latest_image   = None   # latest cv2 image for snapshot
        self.cube_detected  = False

        # ── Waypoints ────────────────────────────────────────────────────────
        self.waypoints = self._load_waypoints()
        self.waypoint_index = 0

        # ── State machine ─────────────────────────────────────────────────────
        self.state      = 'SEARCHING'
        self.start_time = time.time()

        # ── Reported results ──────────────────────────────────────────────────
        self.cube_x      = None
        self.cube_y      = None
        self.return_x    = None
        self.return_y    = None

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(
            f'Autonomous search started — {len(self.waypoints)} waypoints loaded')
        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(f'  Waypoint {i}: {wp}')

        # ── Obstacle avoidance memory ─────────────────────────────
        self.avoiding = False
        self.avoid_direction = 0      # +1 = left, -1 = right

    # ── Waypoint loading ──────────────────────────────────────────────────────

    def _load_waypoints(self):
        """Try PGM feature extraction first, fall back to manual waypoints."""
        if PGM_MAP_PATH is not None and os.path.exists(PGM_MAP_PATH):
            self.get_logger().info('Attempting PGM feature extraction...')
            extracted = extract_cylinder_waypoints(PGM_MAP_PATH, PGM_MAP_YAML)
            if extracted:
                self.get_logger().info(
                    f'Feature extraction succeeded: {len(extracted)} waypoints')
                return extracted
            else:
                self.get_logger().warn(
                    'Feature extraction failed — falling back to manual waypoints')
        else:
            self.get_logger().info('No PGM map path set — using manual waypoints')
        return list(MANUAL_WAYPOINTS)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def scan_callback(self, msg):
        self.latest_scan = msg
        inc     = msg.angle_increment
        arc_r   = math.radians(FRONT_ARC_DEG)
        side_r  = math.radians(90)
        front_i = int(round((LIDAR_OFFSET_RAD - msg.angle_min) / inc))
        half_a  = int(round(arc_r / inc))
        side_a  = int(round(side_r / inc))
        n       = len(msg.ranges)

        def arc_min(lo, hi):
            lo = max(0, lo); hi = min(n - 1, hi)
            vals = [r for r in msg.ranges[lo:hi+1]
                    if msg.range_min < r < msg.range_max]
            return min(vals) if vals else float('inf')

        self.nearest_front = arc_min(front_i - half_a, front_i + half_a)
        self.nearest_left  = arc_min(front_i,          front_i + side_a)
        self.nearest_right = arc_min(front_i - side_a, front_i)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.current_x = pos.x
        self.current_y = pos.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)   # radians

    def image_callback(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        self.latest_image = img
        if self.state != 'SEARCHING':
            return
        hsv  = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, RED_LOW1, RED_HIGH1),
            cv2.inRange(hsv, RED_LOW2, RED_HIGH2))
        if cv2.countNonZero(mask) >= MIN_PIXELS:
            self.cube_detected = True

    # ── Helpers ───────────────────────────────────────────────────────────────

    def stop(self):
        self.cmd_pub.publish(Twist())

    def _publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def _angle_diff(self, target, current):
        """Shortest signed difference between two angles in radians."""
        diff = target - current
        while diff >  math.pi: diff -= 2 * math.pi
        while diff < -math.pi: diff += 2 * math.pi
        return diff

    def _distance_to(self, tx, ty):
        return math.sqrt((tx - self.current_x)**2 + (ty - self.current_y)**2)

    def _estimate_cube_distance(self):
        """
        Return distance to cube using minimum LiDAR range in the
        forward-facing arc (±CUBE_FORWARD_ARC_DEG). Filters inf/nan.
        Falls back to LIDAR_FALLBACK_DIST if arc is empty.
        """
        if self.latest_scan is None:
            return LIDAR_FALLBACK_DIST
        msg     = self.latest_scan
        inc     = msg.angle_increment
        arc_r   = math.radians(CUBE_FORWARD_ARC_DEG)
        front_i = int(round((LIDAR_OFFSET_RAD - msg.angle_min) / inc))
        half_a  = int(round(arc_r / inc))
        n       = len(msg.ranges)
        lo      = max(0, front_i - half_a)
        hi      = min(n - 1, front_i + half_a)
        valid   = [r for r in msg.ranges[lo:hi+1]
                   if msg.range_min < r < msg.range_max
                   and not math.isnan(r) and not math.isinf(r)]
        if not valid:
            self.get_logger().warn(
                'Forward arc empty — using fallback distance for cube estimate')
            return LIDAR_FALLBACK_DIST
        return min(valid)

    def _save_snapshot(self):
        if self.latest_image is None:
            self.get_logger().warn('No image available for snapshot')
            return
        ok = cv2.imwrite(SNAPSHOT_PATH, self.latest_image)
        if ok:
            self.get_logger().info(f'Snapshot saved → {SNAPSHOT_PATH}')
        else:
            self.get_logger().warn(f'cv2.imwrite FAILED — snapshot not saved')


    def _print_forward_lidar_debug(self):
        """
        Print minimum LiDAR distance directly in front of robot
        using the same forward arc used for cube estimation.
        """
        if self.latest_scan is None:
            self.get_logger().warn('No LiDAR scan available')
            return

        msg     = self.latest_scan
        inc     = msg.angle_increment
        arc_r   = math.radians(CUBE_FORWARD_ARC_DEG)

        front_i = int(round((LIDAR_OFFSET_RAD - msg.angle_min) / inc))
        half_a  = int(round(arc_r / inc))

        n  = len(msg.ranges)
        lo = max(0, front_i - half_a)
        hi = min(n - 1, front_i + half_a)

        valid = [
            r for r in msg.ranges[lo:hi+1]
            if msg.range_min < r < msg.range_max
            and not math.isnan(r)
            and not math.isinf(r)
        ]

        if not valid:
            self.get_logger().warn(
                'Forward LiDAR arc empty')
            return

        min_dist = min(valid)

        self.get_logger().info(
            f'FORWARD LIDAR MIN: {min_dist:.3f} m')

    # ── State machine ─────────────────────────────────────────────────────────

    def control_loop(self):

        # ── Time limit check ─────────────────────────────────────────────────
        if self.state == 'SEARCHING':
            self._print_forward_lidar_debug()
            elapsed = time.time() - self.start_time
            if elapsed >= TIME_LIMIT_S:
                self.get_logger().warn(
                    f'Time limit reached ({TIME_LIMIT_S:.0f}s) — transitioning to RETURNING')
                self.stop()
                self.state = 'RETURNING'
                return

        # ── SEARCHING ────────────────────────────────────────────────────────
        if self.state == 'SEARCHING':

            # Cube detected anywhere during navigation
            if self.cube_detected:
                self.stop()
                self.get_logger().info('Red cube detected — transitioning to REPORTING')
                self.state = 'REPORTING'
                return

            # All waypoints visited without detection
            if self.waypoint_index >= len(self.waypoints):
                self.get_logger().warn(
                    'All waypoints visited, cube not found — transitioning to RETURNING')
                self.stop()
                self.state = 'RETURNING'
                return

            target_x, target_y = self.waypoints[self.waypoint_index]
            dist = self._distance_to(target_x, target_y)

            # ── Arrived at waypoint: pause and scan ──────────────────────────
            if dist < WAYPOINT_TOLERANCE:
                self.stop()
                self.get_logger().info(
                    f'Waypoint {self.waypoint_index} reached '
                    f'({target_x:.2f}, {target_y:.2f}) — scanning for cube')
                # cube_detected flag is updated continuously by image_callback;
                # give one extra control cycle for the image to be processed
                self.waypoint_index += 1
                return

            # ── Obstacle avoidance takes priority over waypoint heading ──────
            # ── Start obstacle avoidance ─────────────────────────────
            if self.nearest_front <= AVOID_DISTANCE and not self.avoiding:

                self.avoiding = True

                if self.nearest_left >= self.nearest_right:
                    self.avoid_direction = +1
                    self.get_logger().warn('Obstacle detected — COMMIT LEFT')
                else:
                    self.avoid_direction = -1
                    self.get_logger().warn('Obstacle detected — COMMIT RIGHT')

            # ── Continue obstacle avoidance ──────────────────────────
            if self.avoiding:

                # Keep turning until front is clear
                if self.nearest_front > (AVOID_DISTANCE + 0.10):

                    self.avoiding = False
                    self.get_logger().info('Obstacle cleared')

                else:

                    self._publish_twist(
                        0.0,
                        self.avoid_direction * AVOID_TURN_SPEED
                    )

                    return  # skip normal waypoint control while avoiding

            # ── Steer toward waypoint ─────────────────────────────────────────
            target_angle = math.atan2(
                target_y - self.current_y,
                target_x - self.current_x)
            heading_err = self._angle_diff(target_angle, self.current_yaw)

            if abs(heading_err) > HEADING_TOLERANCE:
                # Rotate to face waypoint
                turn = TURN_SPEED if heading_err > 0 else -TURN_SPEED
                self._publish_twist(0.0, turn)
            else:
                # Drive toward waypoint
                self._publish_twist(FORWARD_SPEED, 0.0)

            self.get_logger().info(
                f'→ WP{self.waypoint_index} ({target_x:.2f},{target_y:.2f}) '
                f'dist={dist:.2f} m  hdg_err={math.degrees(heading_err):.1f}°  '
                f'pos=({self.current_x:.2f},{self.current_y:.2f})')

        # ── REPORTING ────────────────────────────────────────────────────────
        elif self.state == 'REPORTING':
            self.stop()

            # Estimate cube world position from LiDAR distance + current pose
            d = self._estimate_cube_distance()
            self.cube_x = self.current_x + d * math.cos(self.current_yaw)
            self.cube_y = self.current_y + d * math.sin(self.current_yaw)

            self.get_logger().info(
                f'Cube position estimate: '
                f'x={self.cube_x:.3f} m  y={self.cube_y:.3f} m  '
                f'(robot at ({self.current_x:.3f},{self.current_y:.3f}), '
                f'd={d:.3f} m, yaw={math.degrees(self.current_yaw):.1f}°)')

            self._save_snapshot()
            self.state = 'RETURNING'

        # ── RETURNING ────────────────────────────────────────────────────────
        elif self.state == 'RETURNING':
            dx = 0.0 - self.current_x
            dy = 0.0 - self.current_y
            dist = math.sqrt(dx**2 + dy**2)

            if dist < RETURN_TOLERANCE:
                self.stop()
                self.return_x = self.current_x
                self.return_y = self.current_y
                self.get_logger().info(
                    f'Origin reached — x={self.current_x:.3f} y={self.current_y:.3f}')
                self.state = 'DONE'
                return

            target_angle = math.atan2(dy, dx)
            heading_err  = self._angle_diff(target_angle, self.current_yaw)

            if abs(heading_err) > HEADING_TOLERANCE:
                turn = TURN_SPEED if heading_err > 0 else -TURN_SPEED
                self._publish_twist(0.0, turn)
            else:
                self._publish_twist(RETURN_SPEED, 0.0)

            self.get_logger().info(
                f'Returning | dist={dist:.2f} m  '
                f'hdg_err={math.degrees(heading_err):.1f}°')

        # ── DONE ─────────────────────────────────────────────────────────────
        elif self.state == 'DONE':
            self.stop()
            total_time = time.time() - self.start_time
            self.get_logger().info('=' * 50)
            self.get_logger().info('AUTONOMOUS RUN COMPLETE')
            self.get_logger().info(
                f'Cube position  : '
                f'({self.cube_x:.3f}, {self.cube_y:.3f}) m'
                if self.cube_x is not None
                else 'Cube position  : NOT DETECTED')
            self.get_logger().info(
                f'Return position: '
                f'({self.return_x:.3f}, {self.return_y:.3f}) m')
            self.get_logger().info(
                f'Total run time : {total_time:.1f} s')
            self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousSearch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()