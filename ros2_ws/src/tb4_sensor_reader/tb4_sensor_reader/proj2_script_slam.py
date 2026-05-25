#!/usr/bin/env python3
"""
Autonomous Cube Finder — COMPSYS732  (SLAM edition)
Phase 2 autonomous search node using slam_toolbox localisation + nav2.

Prerequisites
─────────────
  • slam_toolbox running in localisation mode (map already built):
      ros2 launch slam_toolbox localization_launch.py  \
           slam_params_file:=<your_params.yaml>
  • nav2 bringup (with the same map):
      ros2 launch nav2_bringup navigation_launch.py    \
           map:=~/Desktop/lab_map.yaml use_sim_time:=false
  • This node:
      ~/ros2_venv/bin/python3 autonomous_search_slam.py

Map files expected
──────────────────
  ~/Desktop/lab_map.pgm        — occupancy grid image
  ~/Desktop/lab_map.yaml       — grid metadata (resolution, origin)
  ~/Desktop/lab_map.data       — slam_toolbox serialised map data
  ~/Desktop/lab_map.posegraph  — slam_toolbox pose-graph

States
──────
  INITIALISING   → wait for nav2 to become active, seed initial pose
  WALL_FOLLOWING → navigate to next waypoint via nav2
  CUBE_FINDING   → spin 360° at waypoint while watching camera
  REPORTING      → record cube pose, save snapshot
  RETURNING      → navigate back to origin via nav2
  DONE           → stop, log results
"""

import rclpy, cv2, math, os, time, struct, subprocess, shlex
import numpy as np

from rclpy.node          import Node
from rclpy.action        import ActionClient
from geometry_msgs.msg   import (Twist, PoseStamped, PoseWithCovarianceStamped,
                                  Quaternion)
from sensor_msgs.msg     import LaserScan, CompressedImage
from nav_msgs.msg        import Odometry
from action_msgs.msg     import GoalStatus
from nav2_msgs.action    import NavigateToPose
from cv_bridge           import CvBridge

# ── Robot namespace ────────────────────────────────────────────────────────────
NAMESPACE = '/T12'

# ── Map file paths ─────────────────────────────────────────────────────────────
PGM_MAP_PATH   = os.path.expanduser('~/Desktop/lab_map.pgm')
PGM_MAP_YAML   = os.path.expanduser('~/Desktop/lab_map.yaml')
POSEGRAPH_PATH = os.path.expanduser('~/Desktop/lab_map.posegraph')

# ── Snapshot output path ───────────────────────────────────────────────────────
SNAPSHOT_PATH = os.path.expanduser('~/Desktop/detection_snapshot.jpg')

# ── Red cube detection ─────────────────────────────────────────────────────────
RED_LOW1   = np.array([0,   120,  70])
RED_HIGH1  = np.array([10,  255, 255])
RED_LOW2   = np.array([170, 120,  70])
RED_HIGH2  = np.array([180, 255, 255])
MIN_PIXELS = 20000

# ── Waypoint approach tolerance ────────────────────────────────────────────────
WAYPOINT_EARLY_RADIUS = 1.5   # m — start spin before nav2 declares arrival

# ── 360° spin ─────────────────────────────────────────────────────────────────
SPIN_SPEED       = 0.3
SPIN_FULL_CIRCLE = 2 * math.pi

# ── LiDAR ─────────────────────────────────────────────────────────────────────
LIDAR_OFFSET_RAD = math.radians(-90)
FRONT_ARC_DEG    = 60

# ── Time limit ─────────────────────────────────────────────────────────────────
TIME_LIMIT_S = 420.0

# ── Blob filter for cylinder extraction ───────────────────────────────────────
MIN_BLOB_AREA = 4
MAX_BLOB_AREA = 30

# ── Manual waypoint fallback ───────────────────────────────────────────────────
MANUAL_WAYPOINTS = [
    (-2.3, -2.3),
    ( 1.5,  0.5),
    ( 1.5,  1.0),
]


# ──────────────────────────────────────────────────────────────────────────────
# Pure utility functions
# ──────────────────────────────────────────────────────────────────────────────

def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q


def make_pose_stamped(x, y, yaw, frame='map') -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.position.z = 0.0
    ps.pose.orientation = yaw_to_quaternion(yaw)
    return ps


def angle_diff(target: float, current: float) -> float:
    d = target - current
    while d >  math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


def extract_cylinder_waypoints(pgm_path, yaml_path):
    """Return list of (world_x, world_y) for cylinder blobs inside the arena."""
    try:
        import yaml
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        resolution = meta['resolution']
        origin     = meta['origin']

        img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            print(f'[EXTRACT] Cannot read {pgm_path}')
            return None
        h, w = img.shape
        print(f'[EXTRACT] Map {w}x{h} px ({w*resolution:.1f}x{h*resolution:.1f} m)')

        _, binary = cv2.threshold(img, 50, 255, cv2.THRESH_BINARY_INV)
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            binary, connectivity=8)

        wall_label = 1 + int(np.argmax(
            [stats[i, cv2.CC_STAT_AREA] for i in range(1, num_labels)]))
        wall_mask = (labels == wall_label).astype(np.uint8) * 255
        contours, _ = cv2.findContours(wall_mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        border = max(contours, key=cv2.contourArea)

        def inside(px, py):
            return cv2.pointPolygonTest(border, (float(px), float(py)), False) >= 0

        waypoints = []
        for i in range(1, num_labels):
            if i == wall_label:
                continue
            area   = stats[i, cv2.CC_STAT_AREA]
            blob_w = stats[i, cv2.CC_STAT_WIDTH]
            blob_h = stats[i, cv2.CC_STAT_HEIGHT]
            cx, cy = centroids[i]
            if area < MIN_BLOB_AREA or area > MAX_BLOB_AREA: continue
            if blob_w < 2 or blob_h < 2: continue
            ar = blob_w / blob_h
            if ar < 0.5 or ar > 2.0: continue
            if not inside(cx, cy): continue
            wx = origin[0] + cx * resolution
            wy = origin[1] + (h - cy) * resolution
            waypoints.append((round(wx, 3), round(wy, 3)))
            print(f'[EXTRACT] Cylinder world ({wx:.3f}, {wy:.3f})')

        # Debug image
        try:
            S = 6
            dbg = cv2.cvtColor(
                cv2.resize(binary, (w*S, h*S), interpolation=cv2.INTER_NEAREST),
                cv2.COLOR_GRAY2BGR)
            cv2.drawContours(dbg, [(border * S).astype(np.int32)], -1, (255,165,0), 1)
            for wx, wy in waypoints:
                px = int((wx - origin[0]) / resolution) * S
                py = int(h - (wy - origin[1]) / resolution) * S
                cv2.circle(dbg, (px, py), 10, (0, 0, 255), -1)
            out = os.path.splitext(pgm_path)[0] + '_features_slam.jpg'
            cv2.imwrite(out, dbg)
            print(f'[EXTRACT] Debug image -> {out}')
        except Exception as e:
            print(f'[EXTRACT] Debug image failed: {e}')

        return waypoints
    except Exception as e:
        print(f'[EXTRACT] Exception: {e}')
        return None


def read_initial_pose_from_posegraph(posegraph_path):
    """
    Heuristic parser for slam_toolbox .posegraph binary.
    Scans for the last plausible (x, y, theta) triple.
    Returns (0, 0, 0) on any failure.
    """
    try:
        with open(posegraph_path, 'rb') as f:
            raw = f.read()

        DOUBLE = struct.calcsize('d')
        doubles = []
        for off in range(0, len(raw) - DOUBLE + 1, DOUBLE):
            val, = struct.unpack_from('d', raw, off)
            if not (math.isnan(val) or math.isinf(val)) and abs(val) < 1e6:
                doubles.append(val)

        for i in range(len(doubles) - 3, -1, -1):
            x, y, theta = doubles[i], doubles[i+1], doubles[i+2]
            if abs(x) < 50 and abs(y) < 50 and abs(theta) <= math.pi:
                print(f'[POSEGRAPH] Last pose: x={x:.3f} y={y:.3f} '
                      f'theta={math.degrees(theta):.1f} deg')
                return x, y, theta

        print('[POSEGRAPH] Could not parse pose — defaulting to origin')
        return 0.0, 0.0, 0.0
    except Exception as e:
        print(f'[POSEGRAPH] Read failed ({e}) — defaulting to origin')
        return 0.0, 0.0, 0.0


# ──────────────────────────────────────────────────────────────────────────────
# Main node
# ──────────────────────────────────────────────────────────────────────────────

class AutonomousSearchSlam(Node):

    def __init__(self):
        super().__init__('autonomous_search_slam')
        self.bridge = CvBridge()

        # ── Publishers ───────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, f'{NAMESPACE}/cmd_vel', 10)
        self.init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # ── nav2 action client ────────────────────────────────────────────────
        # The node auto-detects the correct action server name in INITIALISING.
        # Candidates tried in order:
        #   1. /navigate_to_pose         (global nav2 bringup)
        #   2. /T12/navigate_to_pose     (namespaced nav2)
        self._nav_action_name  = '/navigate_to_pose'
        self._nav_client       = ActionClient(self, NavigateToPose,
                                              self._nav_action_name)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(LaserScan, f'{NAMESPACE}/scan',
                                 self._scan_cb, 10)
        self.create_subscription(CompressedImage,
                                 f'{NAMESPACE}/oakd/rgb/image_raw/compressed',
                                 self._image_cb, 10)
        # Prefer SLAM pose (/pose from slam_toolbox); fall back to /odom
        self.create_subscription(PoseWithCovarianceStamped, '/pose',
                                 self._slam_pose_cb, 10)
        self.create_subscription(Odometry, f'{NAMESPACE}/odom',
                                 self._odom_cb, 10)

        # ── Sensor / localisation state ───────────────────────────────────────
        self.current_x     = 0.0
        self.current_y     = 0.0
        self.current_yaw   = 0.0
        self.pose_source   = 'odom'
        self.nearest_front = float('inf')
        self.nearest_left  = float('inf')
        self.nearest_right = float('inf')
        self.latest_image  = None
        self.cube_detected = False

        # ── nav2 goal tracking ────────────────────────────────────────────────
        self._goal_handle      = None
        self._goal_in_flight   = False
        self._nav2_ready       = False
        self._nav2_check_count = 0   # ticks spent in INITIALISING

        # ── Spin bookkeeping ──────────────────────────────────────────────────
        self.spin_dir         = 1.0
        self.spin_accumulated = 0.0
        self.spin_last_yaw    = None

        # ── Waypoints (loaded once; posegraph parsed once) ────────────────────
        self._init_x, self._init_y, self._init_yaw = \
            read_initial_pose_from_posegraph(POSEGRAPH_PATH)
        self.waypoints      = self._load_waypoints()
        self.waypoint_index = 0

        # ── State machine ─────────────────────────────────────────────────────
        self.state      = 'INITIALISING'
        self.start_time = time.time()

        # ── Results ───────────────────────────────────────────────────────────
        self.cube_x   = None
        self.cube_y   = None
        self.return_x = None
        self.return_y = None

        self.timer = self.create_timer(0.1, self._control_loop)
        self.get_logger().info(
            f'SLAM search node started — {len(self.waypoints)} waypoints  '
            f'init_pose=({self._init_x:.2f},{self._init_y:.2f},'
            f'{math.degrees(self._init_yaw):.1f}deg)')

    # ── Waypoint loading ──────────────────────────────────────────────────────

    def _load_waypoints(self):
        wps = extract_cylinder_waypoints(PGM_MAP_PATH, PGM_MAP_YAML)
        if not wps:
            self.get_logger().warn('Cylinder extraction failed — using manual waypoints')
            return list(MANUAL_WAYPOINTS)
        self.get_logger().info(f'Loaded {len(wps)} cylinder waypoints')
        return wps

    # ── Subscribers ───────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        inc   = msg.angle_increment
        arc_r = math.radians(FRONT_ARC_DEG)
        side_r = math.radians(90)
        fi    = int(round((LIDAR_OFFSET_RAD - msg.angle_min) / inc))
        ha    = int(round(arc_r  / inc))
        sa    = int(round(side_r / inc))
        n     = len(msg.ranges)

        def arc_min(lo, hi):
            lo = max(0, lo); hi = min(n-1, hi)
            vals = [r for r in msg.ranges[lo:hi+1]
                    if msg.range_min < r < msg.range_max]
            return min(vals) if vals else float('inf')

        self.nearest_front = arc_min(fi - ha, fi + ha)
        self.nearest_left  = arc_min(fi,      fi + sa)
        self.nearest_right = arc_min(fi - sa, fi)

    def _slam_pose_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        self.current_x = p.x
        self.current_y = p.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2.0 * (q.w*q.z + q.x*q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z))
        self.pose_source = 'slam'

    def _odom_cb(self, msg: Odometry):
        if self.pose_source == 'slam':
            return
        p = msg.pose.pose.position
        self.current_x = p.x
        self.current_y = p.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2.0 * (q.w*q.z + q.x*q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z))

    def _image_cb(self, msg: CompressedImage):
        img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        self.latest_image = img
        if self.state != 'CUBE_FINDING':
            return
        hsv  = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, RED_LOW1, RED_HIGH1),
            cv2.inRange(hsv, RED_LOW2, RED_HIGH2))
        if cv2.countNonZero(mask) >= MIN_PIXELS:
            self.cube_detected = True

    # ── nav2 helpers ──────────────────────────────────────────────────────────

    def _send_nav_goal(self, x, y, yaw=0.0):
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('[NAV2] Action server not ready when sending goal')
            return

        goal = NavigateToPose.Goal()
        goal.pose = make_pose_stamped(x, y, yaw)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.behavior_tree = ''

        self.get_logger().info(
            f'[NAV2] Goal -> ({x:.2f},{y:.2f}) yaw={math.degrees(yaw):.0f}deg')

        future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._nav_feedback_cb)
        future.add_done_callback(self._nav_goal_response_cb)
        self._goal_in_flight = True

    def _nav_goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('[NAV2] Goal rejected')
            self._goal_in_flight = False
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._nav_result_cb)

    def _nav_feedback_cb(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        # Early arrival check: cancel goal and start spin
        if self.state == 'WALL_FOLLOWING' and \
                self.waypoint_index < len(self.waypoints):
            tx, ty = self.waypoints[self.waypoint_index]
            if self._distance_to(tx, ty) < WAYPOINT_EARLY_RADIUS:
                self.get_logger().info(
                    f'[NAV2] Within {WAYPOINT_EARLY_RADIUS} m of WP'
                    f'{self.waypoint_index} — cancelling and spinning')
                self._cancel_goal()
                self._begin_spin()

    def _nav_result_cb(self, future):
        status = future.result().status
        self._goal_in_flight = False
        self._goal_handle    = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('[NAV2] Goal succeeded')
            if self.state == 'WALL_FOLLOWING':
                self._begin_spin()
            elif self.state == 'RETURNING':
                self._finish_return()
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('[NAV2] Goal cancelled (expected)')
        else:
            self.get_logger().warn(
                f'[NAV2] Goal failed status={status} — skipping waypoint')
            if self.state == 'WALL_FOLLOWING':
                self.waypoint_index += 1
                self._send_next_waypoint()

    def _cancel_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

    # ── Spin helpers ──────────────────────────────────────────────────────────

    def _begin_spin(self):
        self.stop()
        self.spin_dir = 1.0 if self.nearest_left >= self.nearest_right else -1.0
        self.spin_accumulated = 0.0
        self.spin_last_yaw    = self.current_yaw
        self.cube_detected    = False
        label = 'CCW' if self.spin_dir > 0 else 'CW'
        self.get_logger().info(
            f'[SPIN] WP{self.waypoint_index} dir={label}  '
            f'L={self.nearest_left:.2f} m  R={self.nearest_right:.2f} m')
        self.state = 'CUBE_FINDING'

    def _finish_return(self):
        self.stop()
        self.return_x = self.current_x
        self.return_y = self.current_y
        self.get_logger().info(
            f'[RETURN] Origin reached ({self.current_x:.3f},{self.current_y:.3f})')
        self.state = 'DONE'

    # ── Waypoint sequencing ───────────────────────────────────────────────────

    def _send_next_waypoint(self):
        if self.waypoint_index >= len(self.waypoints):
            self.get_logger().warn('All waypoints visited — returning to origin')
            self.state = 'RETURNING'
            self._send_nav_goal(0.0, 0.0, 0.0)
            return
        tx, ty = self.waypoints[self.waypoint_index]
        yaw = math.atan2(ty - self.current_y, tx - self.current_x)
        self._send_nav_goal(tx, ty, yaw)

    # ── General helpers ───────────────────────────────────────────────────────

    def stop(self):
        self.cmd_pub.publish(Twist())

    def _publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x  = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def _distance_to(self, tx, ty):
        return math.sqrt((tx - self.current_x)**2 + (ty - self.current_y)**2)

    def _publish_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.pose.pose.position.x  = float(x)
        msg.pose.pose.position.y  = float(y)
        msg.pose.pose.position.z  = 0.0
        msg.pose.pose.orientation = yaw_to_quaternion(yaw)
        cov = [0.0] * 36
        cov[0]  = 0.05   # x variance
        cov[7]  = 0.05   # y variance
        cov[35] = 0.05   # yaw variance
        msg.pose.covariance = cov
        self.init_pose_pub.publish(msg)
        self.get_logger().info(
            f'[INIT] Published /initialpose '
            f'({x:.3f},{y:.3f},{math.degrees(yaw):.1f}deg)')

    def _save_snapshot(self):
        if self.latest_image is None:
            self.get_logger().warn('No image for snapshot')
            return
        ok = cv2.imwrite(SNAPSHOT_PATH, self.latest_image)
        self.get_logger().info(
            f'Snapshot {"saved" if ok else "FAILED"} -> {SNAPSHOT_PATH}')

    # ── Main control loop ─────────────────────────────────────────────────────

    def _control_loop(self):

        # ── Global time limit ─────────────────────────────────────────────────
        if self.state in ('WALL_FOLLOWING', 'CUBE_FINDING'):
            if time.time() - self.start_time >= TIME_LIMIT_S:
                self.get_logger().warn('Time limit reached — forcing RETURNING')
                self._cancel_goal()
                self.stop()
                self.state = 'RETURNING'
                self._send_nav_goal(0.0, 0.0, 0.0)
                return

        # ══ INITIALISING ═════════════════════════════════════════════════════
        if self.state == 'INITIALISING':
            self._nav2_check_count += 1

            # Publish initial pose once on the very first tick only
            if self._nav2_check_count == 1:
                self._publish_initial_pose(
                    self._init_x, self._init_y, self._init_yaw)

            # Every ~2 s: list all available action servers so the user can
            # see what's running, and auto-try the namespaced variant.
            if self._nav2_check_count % 20 == 0:
                try:
                    result = subprocess.run(
                        shlex.split('ros2 action list'),
                        capture_output=True, text=True, timeout=3.0)
                    actions = result.stdout.strip()
                    self.get_logger().info(
                        '[INIT] ros2 action list:\n' +
                        (actions if actions else
                         '  (empty — is nav2 launched?)'))
                except Exception as e:
                    self.get_logger().warn(f'[INIT] action list failed: {e}')

                # Auto-detect: try namespaced action server
                ns = f'{NAMESPACE}/navigate_to_pose'
                if self._nav_action_name != ns:
                    probe = ActionClient(self, NavigateToPose, ns)
                    if probe.wait_for_server(timeout_sec=0.3):
                        self.get_logger().info(
                            f'[INIT] Found nav2 at {ns} — switching')
                        self._nav_client.destroy()
                        self._nav_client       = probe
                        self._nav_action_name  = ns
                    else:
                        probe.destroy()

            # Check whichever client is currently active
            if self._nav_client.wait_for_server(timeout_sec=0.3):
                self.get_logger().info(
                    f'[INIT] nav2 ready at "{self._nav_action_name}"'
                    ' — starting search')
                self._nav2_ready = True
                self.state = 'WALL_FOLLOWING'
                self._send_next_waypoint()
            elif self._nav2_check_count % 10 == 0:
                self.get_logger().info(
                    f'[INIT] Waiting for "{self._nav_action_name}"… '
                    f'({self._nav2_check_count // 10} s)  '
                    f'pose_src={self.pose_source}')
            return

        # ══ WALL_FOLLOWING ════════════════════════════════════════════════════
        elif self.state == 'WALL_FOLLOWING':
            if not self._goal_in_flight:
                self.get_logger().warn('[WALL_FOLLOWING] Goal dropped — re-sending')
                self._send_next_waypoint()

            self.get_logger().info(
                f'[WALL_FOLLOWING] WP{self.waypoint_index}  '
                f'src={self.pose_source}  '
                f'pos=({self.current_x:.2f},{self.current_y:.2f})  '
                f'front={self.nearest_front:.2f}m',
                throttle_duration_sec=1.0)

        # ══ CUBE_FINDING ══════════════════════════════════════════════════════
        elif self.state == 'CUBE_FINDING':

            if self.cube_detected:
                self.stop()
                self.get_logger().info('[CUBE_FINDING] Cube detected -> REPORTING')
                self.state = 'REPORTING'
                return

            if self.spin_last_yaw is not None:
                delta = abs(angle_diff(self.current_yaw, self.spin_last_yaw))
                self.spin_accumulated += delta
            self.spin_last_yaw = self.current_yaw

            self._publish_twist(0.0, self.spin_dir * SPIN_SPEED)

            self.get_logger().info(
                f'[CUBE_FINDING] WP{self.waypoint_index}  '
                f'rotated={math.degrees(self.spin_accumulated):.1f}deg',
                throttle_duration_sec=0.5)

            if self.spin_accumulated >= SPIN_FULL_CIRCLE:
                self.stop()
                self.get_logger().info(
                    f'[CUBE_FINDING] 360 done at WP{self.waypoint_index}'
                    ' — no cube, next waypoint')
                self.waypoint_index += 1
                self.state = 'WALL_FOLLOWING'
                self._send_next_waypoint()

        # ══ REPORTING ════════════════════════════════════════════════════════
        elif self.state == 'REPORTING':
            self.stop()
            self.cube_x = self.current_x
            self.cube_y = self.current_y
            self.get_logger().info(
                f'[REPORTING] Cube at ({self.cube_x:.3f},{self.cube_y:.3f}) m  '
                f'yaw={math.degrees(self.current_yaw):.1f}deg  '
                f'src={self.pose_source}')
            self._save_snapshot()
            self.state = 'RETURNING'
            self._send_nav_goal(0.0, 0.0, 0.0)

        # ══ RETURNING ════════════════════════════════════════════════════════
        elif self.state == 'RETURNING':
            if not self._goal_in_flight:
                self.get_logger().warn('[RETURNING] No goal — re-sending origin')
                self._send_nav_goal(0.0, 0.0, 0.0)
            self.get_logger().info(
                f'[RETURNING] dist={self._distance_to(0,0):.2f}m  '
                f'pos=({self.current_x:.2f},{self.current_y:.2f})',
                throttle_duration_sec=1.0)

        # ══ DONE ═════════════════════════════════════════════════════════════
        elif self.state == 'DONE':
            self.stop()
            elapsed = time.time() - self.start_time
            self.get_logger().info('=' * 52)
            self.get_logger().info('AUTONOMOUS SLAM RUN COMPLETE')
            self.get_logger().info(
                (f'Cube position  : ({self.cube_x:.3f},{self.cube_y:.3f}) m')
                if self.cube_x is not None
                else 'Cube position  : NOT DETECTED')
            if self.return_x is not None:
                self.get_logger().info(
                    f'Return position: ({self.return_x:.3f},{self.return_y:.3f}) m')
            self.get_logger().info(f'Total run time : {elapsed:.1f} s')
            self.get_logger().info(f'Pose source    : {self.pose_source}')
            self.get_logger().info('=' * 52)
            self.timer.cancel()


# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousSearchSlam()
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