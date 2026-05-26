#!/usr/bin/env python3
"""
Autonomous Cube Finder — COMPSYS732
Phase 2 autonomous search node.

States: WALL_FOLLOWING → CUBE_FINDING → REPORTING → RETURNING → DONE

To run:
    ~/ros2_venv/bin/python3 -m tb4_sensor_reader.autonomous_search
"""

import rclpy, cv2, math, os, time, sys
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, CompressedImage
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

# ── Robot namespace ────────────────────────────────────────────────────────────
NAMESPACE = '/T13'

# ── Motion parameters ──────────────────────────────────────────────────────────
FORWARD_SPEED    = 0.15      # m/s — waypoint navigation
TURN_SPEED       = 0.8       # rad/s — waypoint navigation turns
AVOID_TURN_SPEED = 0.5       # rad/s — obstacle avoidance turns

# ── Tolerances ─────────────────────────────────────────────────────────────────
WAYPOINT_TOLERANCE  = 1    # m — close enough to waypoint to advance
RETURN_TOLERANCE    = 0.15   # m — close enough to origin to stop

# ── Obstacle avoidance ─────────────────────────────────────────────────────────
AVOID_DISTANCE      = 0.3    # m — obstacle trigger distance
FRONT_ARC_DEG       = 60     # degrees — front detection arc width
LIDAR_OFFSET_DEG    = -90    # degrees — LiDAR mounting offset
LIDAR_OFFSET_RAD    = math.radians(LIDAR_OFFSET_DEG)

# ── Time limit ─────────────────────────────────────────────────────────────────
TIME_LIMIT_S = 420.0         # 7 minutes — force RETURNING if cube not found

# ── Red cube detection ─────────────────────────────────────────────────────────
RED_LOW1   = np.array([0,   150, 100])
RED_HIGH1  = np.array([8,   255, 255])
RED_LOW2   = np.array([172, 150, 100])
RED_HIGH2  = np.array([180, 255, 255])
MIN_PIXELS = 5000

# ── Snapshot output path ───────────────────────────────────────────────────────
SNAPSHOT_PATH = os.path.expanduser('~/Desktop/detection_snapshot.jpg')

# ── LiDAR cube distance estimation ────────────────────────────────────────────
CUBE_FORWARD_ARC_DEG = 15    # degrees either side of forward for distance estimate
LIDAR_FALLBACK_DIST  = 0.30  # m — used if forward arc returns no valid ranges

# ── Perimeter following ────────────────────────────────────────────────────────
PERIMETER_WALL_DIST  = 0.35  # m — desired distance from the left wall
PERIMETER_KP         = 1.2   # proportional gain for wall-following correction
PERIMETER_SIDE_ARC   = 20    # degrees either side of the 90° left beam

# ── 360° spin (CUBE_FINDING state) ────────────────────────────────────────────
SPIN_SPEED       = 1       # rad/s — rotation speed during cube-finding spin
SPIN_FULL_CIRCLE = 2 * math.pi  # radians — one complete revolution

# ── Map paths ─────────────────────────────────────────────────────────────────
PGM_MAP_PATH = os.path.expanduser('~/Desktop/lab_map_c_2.pgm')
PGM_MAP_YAML = os.path.expanduser('~/Desktop/lab_map_c_2.yaml')


CUBE_SEARCH_STEP_M   = 0.20   # m — nudge forward each retry
CUBE_SEARCH_MAX_SPINS = 1     # full 360s before nudging

STUCK_CHECK_WINDOW_S  = 10   # seconds — history window for stuck detection
STUCK_MIN_DIST_M      = 0.05  # m — if less than this in window, consider stuck

STUCK_BACKUP_SPEED = -0.15     # m/s — reverse speed when unsticking
STUCK_BACKUP_DIST  = 0.30      # m — distance to reverse
STUCK_BACKUP_TIMEOUT_S = 4.0

STUCK_EXIT_TURN_SPEED = 0.4    # rad/s — turn after backup to unstick
STUCK_EXIT_TURN_DUR_S = 1.0    # seconds — how long to turn

# ── Manual waypoint fallback ───────────────────────────────────────────────────
MANUAL_WAYPOINTS = [
    (-2.3, -2.3),
    (1.5, 0.5),
    (1.5, 1.0),
]


# ── Cylinder feature extraction ────────────────────────────────────────────────

def extract_cylinder_waypoints(pgm_path, yaml_path):
    try:
        import yaml
        with open(yaml_path, 'r') as f:
            meta = yaml.safe_load(f)
        resolution = meta.get('resolution')
        origin     = meta.get('origin')
        print(f'[EXTRACT] YAML loaded — resolution={resolution} origin={origin}')

        img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            print(f'[EXTRACT] cv2.imread failed — {pgm_path}')
            return None
        h, w = img.shape
        print(f'[EXTRACT] PGM loaded — size {w}x{h} px  '
              f'(arena ~{w*resolution:.1f}x{h*resolution:.1f} m)')

        _, binary = cv2.threshold(img, 50, 255, cv2.THRESH_BINARY_INV)

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            binary, connectivity=8)
        print(f'[EXTRACT] Connected components: {num_labels - 1} (excluding background)')
        for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA]
            cx, cy = centroids[i]
            print(f'[EXTRACT]   Component {i}: area={area}  centroid=({cx:.1f},{cy:.1f})')

        wall_label = 1 + int(np.argmax(
            [stats[i, cv2.CC_STAT_AREA] for i in range(1, num_labels)]))
        wall_area  = stats[wall_label, cv2.CC_STAT_AREA]
        print(f'[EXTRACT] Wall component: label={wall_label} area={wall_area}')

        # ── Build a convex hull (or contour) around the wall component ──────────
        wall_mask = (labels == wall_label).astype(np.uint8) * 255
        contours, _ = cv2.findContours(wall_mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        # Use the largest contour as the border polygon
        border_contour = max(contours, key=cv2.contourArea)
        print(f'[EXTRACT] Border contour points: {len(border_contour)}')

        def is_inside_border(px, py):
            """Returns True if pixel coordinate (px, py) is inside the wall contour."""
            result = cv2.pointPolygonTest(border_contour, (float(px), float(py)), False)
            return result >= 0  # 1 = inside, 0 = on edge, -1 = outside

        MIN_BLOB_AREA = 4
        MAX_BLOB_AREA = 30
        waypoints = []

        for i in range(1, num_labels):
            if i == wall_label:
                continue
            area   = stats[i, cv2.CC_STAT_AREA]
            blob_w = stats[i, cv2.CC_STAT_WIDTH]
            blob_h = stats[i, cv2.CC_STAT_HEIGHT]
            cx, cy = centroids[i]
            print(f'[EXTRACT]   Non-wall component {i}: area={area} dim={blob_w}x{blob_h} '
                  f'centroid=({cx:.1f},{cy:.1f})', end='')

            if area < MIN_BLOB_AREA or area > MAX_BLOB_AREA:
                print(f' → rejected (area {area} out of bounds {MIN_BLOB_AREA}-{MAX_BLOB_AREA})')
                continue
            if blob_w < 2 or blob_h < 2:
                print(f' → rejected (dimensions {blob_w}x{blob_h} too narrow)')
                continue
            aspect_ratio = float(blob_w) / float(blob_h)
            if aspect_ratio < 0.5 or aspect_ratio > 2.0:
                print(f' → rejected (aspect ratio {aspect_ratio:.2f} not circular)')
                continue

            # ── NEW: reject cylinders outside the wall border ────────────────
            if not is_inside_border(cx, cy):
                print(f' → rejected (outside wall border)')
                continue

            world_x = origin[0] + cx * resolution
            world_y = origin[1] + (h - cy) * resolution
            waypoints.append((round(world_x, 3), round(world_y, 3)))
            print(f' → accepted  world=({world_x:.3f},{world_y:.3f})')

        try:
            SCALE = 8
            debug_img = cv2.cvtColor(
                cv2.resize(binary, (w * SCALE, h * SCALE),
                           interpolation=cv2.INTER_NEAREST),
                cv2.COLOR_GRAY2BGR)

            # Draw the border contour on the debug image
            scaled_contour = (border_contour * SCALE).astype(np.int32)
            cv2.drawContours(debug_img, [scaled_contour], -1, (255, 165, 0), 1)

            for (wx, wy) in waypoints:
                px_d = int((wx - origin[0]) / resolution) * SCALE
                py_d = int(h - (wy - origin[1]) / resolution) * SCALE
                cv2.circle(debug_img, (px_d, py_d), 12, (0, 0, 255), 2)
                cv2.circle(debug_img, (px_d, py_d),  3, (0, 0, 255), -1)
                cv2.putText(debug_img, f'({wx:.2f},{wy:.2f})',
                            (px_d + 8, py_d - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

            debug_path = os.path.splitext(pgm_path)[0] + '_features.jpg'
            ok = cv2.imwrite(debug_path, debug_img)
            print(f'[EXTRACT] Debug image saved → {debug_path} (ok={ok})')
        except Exception as e:
            print(f'[EXTRACT] Debug image failed: {e}')

        return waypoints
    except Exception as e:
        print(f'[EXTRACT] Unexpected exception: {e}')
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
        self.current_x         = 0.0
        self.current_y         = 0.0
        self.current_yaw       = 0.0
        self.nearest_front     = float('inf')
        self.nearest_left      = float('inf')
        self.nearest_right     = float('inf')
        self.nearest_left_side = float('inf')
        self.latest_scan       = None
        self.latest_image      = None
        self.cube_detected     = False
        self.spin_dir         = 1.0   # default CCW; overwritten by _start_spin
        self.spin_count        = 0     # spins completed at current position
        self.nudge_start_x     = None  # position when nudge began
        self.nudge_start_y     = None
        self.nudging           = False  # True while driving the 20cm step
        self.pos_history       = []    # list of (timestamp, x, y)
        self.unsticking        = False
        self.unstick_phase     = None   # 'TURN1', 'DRIVE', 'TURN2'
        self.unstick_turn_acc  = 0.0
        self.unstick_turn_last_yaw = None
        self.unstick_drive_start_x = None
        self.unstick_drive_start_y = None
        self.backup_start_x    = None
        self.backup_start_y    = None
        self.unstick_backup_start_t = 0.0
        self.unstick_exit_turn_start_t = None  # set when exit turn begins

        # ── Waypoints ────────────────────────────────────────────────────────
        self.waypoints      = self._load_waypoints()
        self.waypoint_index = 0

        # ── 360° spin bookkeeping ─────────────────────────────────────────────
        # spin_start_yaw  — yaw when the spin began
        # spin_accumulated — total absolute rotation so far (always positive)
        # spin_last_yaw   — yaw at the previous control tick (for delta calc)
        self.spin_start_yaw   = None
        self.spin_accumulated = 0.0
        self.spin_last_yaw    = None

        # ── State machine ─────────────────────────────────────────────────────
        #   WALL_FOLLOWING → perimeter-follow toward current waypoint
        #   CUBE_FINDING   → spin 360° at the waypoint, scanning for the cube
        #   REPORTING      → log cube location, save snapshot
        #   RETURNING      → drive back to origin
        #   DONE           → stop and report results

        # ── Resume mode from command-line argument ────────────────────────────────
        resume_mode = None
        for arg in sys.argv[1:]:
            if arg.startswith('--resume='):
                resume_mode = arg.split('=', 1)[1].upper()

        if resume_mode == 'RETURN':
            self.state = 'RETURNING'
            self.get_logger().info('Resume mode: RETURNING — navigating directly to origin')
        elif resume_mode == 'SEARCH':
            # Jump to next waypoint (skip already-visited ones if desired)
            self.state = 'WALL_FOLLOWING'
            self.get_logger().info(f'Resume mode: SEARCH — resuming from waypoint {self.waypoint_index}')
        else:
            self.state = 'WALL_FOLLOWING'
            self.get_logger().info('Normal start — beginning from waypoint 0')

        self.start_time = time.time()

        # ── Reported results ──────────────────────────────────────────────────
        self.cube_x   = None
        self.cube_y   = None
        self.return_x = None
        self.return_y = None

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(
            f'Autonomous search started — {len(self.waypoints)} waypoints loaded')
        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(f'  Waypoint {i}: {wp}')

    def _record_position(self):
        """Call every control tick to maintain a rolling position history."""
        now = time.time()
        self.pos_history.append((now, self.current_x, self.current_y))
        # Trim entries older than the window
        cutoff = now - STUCK_CHECK_WINDOW_S
        self.pos_history = [(t, x, y) for t, x, y in self.pos_history if t >= cutoff]

    def _is_stuck(self):
        if len(self.pos_history) < 2:
            return False
        # Don't trigger until the window is actually full
        window_duration = self.pos_history[-1][0] - self.pos_history[0][0]
        if window_duration < STUCK_CHECK_WINDOW_S * 0.9:
            return False
        total = 0.0
        for i in range(1, len(self.pos_history)):
            _, x0, y0 = self.pos_history[i - 1]
            _, x1, y1 = self.pos_history[i]
            total += math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
        return total < STUCK_MIN_DIST_M

    def _handle_stuck(self):
        """
        Unstick by reversing 30 cm.
        Returns True while unsticking (caller must return immediately).
        """
        if self.unsticking:
            elapsed = time.time() - self.unstick_backup_start_t   # add this

            dist = math.sqrt(
                (self.current_x - self.unstick_drive_start_x) ** 2 +
                (self.current_y - self.unstick_drive_start_y) ** 2)
            self.get_logger().info(
                f'[STUCK-BACKUP] reversed={dist:.2f}/{STUCK_BACKUP_DIST:.2f} m')

            if dist >= STUCK_BACKUP_DIST or elapsed >= STUCK_BACKUP_TIMEOUT_S:
                self.stop()
                self.unsticking = False
                self.unstick_exit_turn_start_t = time.time()
                self.pos_history.clear()
                self.get_logger().warn(
                    f'[STUCK] Backup ended — dist={dist:.2f} m  elapsed={elapsed:.1f} s — starting exit turn')
            else:
                self._publish_twist(STUCK_BACKUP_SPEED, 0.3)
            return True

        if self._is_stuck():
            self.get_logger().warn('[STUCK] Detected — reversing')
            self.unsticking            = True
            self.unstick_backup_start_t = time.time()   # ← add this line
            self.unstick_drive_start_x = self.current_x
            self.unstick_drive_start_y = self.current_y
            self._publish_twist(STUCK_BACKUP_SPEED, 0.0)
            return True
        
        # ── Exit turn after backup ────────────────────────────────────────────
        if self.unstick_exit_turn_start_t is not None:
            turn_elapsed = time.time() - self.unstick_exit_turn_start_t
            if turn_elapsed < STUCK_EXIT_TURN_DUR_S:
                self._publish_twist(0.0, STUCK_EXIT_TURN_SPEED)
                self.get_logger().info(
                    f'[STUCK-EXIT-TURN] {turn_elapsed:.1f}/{STUCK_EXIT_TURN_DUR_S:.1f} s')
                return True
            else:
                self.unstick_exit_turn_start_t = None
                self.get_logger().info('[STUCK] Exit turn done — resuming normally')

        return False

    # ── Waypoint loading ──────────────────────────────────────────────────────

    def _load_waypoints(self):
        if PGM_MAP_PATH is None or not os.path.exists(PGM_MAP_PATH):
            self.get_logger().info('No PGM map found — using manual waypoints')
            return list(MANUAL_WAYPOINTS)

        self.get_logger().info('Extracting cylinder positions from map...')
        cylinders = extract_cylinder_waypoints(PGM_MAP_PATH, PGM_MAP_YAML)
        if not cylinders:
            self.get_logger().warn(
                'Cylinder extraction failed — falling back to manual waypoints')
            return list(MANUAL_WAYPOINTS)

        self.get_logger().info(
            f'Found {len(cylinders)} cylinders — using as direct waypoints:')
        for i, cyl in enumerate(cylinders):
            self.get_logger().info(f'  [{i}] {cyl}')

        try:
            import yaml
            with open(PGM_MAP_YAML, 'r') as f:
                meta = yaml.safe_load(f)
            resolution = meta['resolution']
            origin     = meta['origin']

            img = cv2.imread(PGM_MAP_PATH, cv2.IMREAD_GRAYSCALE)
            if img is not None:
                h, w = img.shape
                SCALE = 6
                debug_img = cv2.cvtColor(
                    cv2.resize(img, (w * SCALE, h * SCALE),
                               interpolation=cv2.INTER_NEAREST),
                    cv2.COLOR_GRAY2BGR)

                def w2p_scaled(wx, wy):
                    px = int((wx - origin[0]) / resolution)
                    py = int(h - (wy - origin[1]) / resolution)
                    return (max(0, min(w - 1, px)) * SCALE,
                            max(0, min(h - 1, py)) * SCALE)

                origin_px = w2p_scaled(0.0, 0.0)
                cv2.circle(debug_img, origin_px, 6, (255, 0, 0), -1)
                cv2.arrowedLine(debug_img, origin_px, w2p_scaled(0.4, 0.0),
                                (0, 0, 255), 3, tipLength=0.2)
                cv2.arrowedLine(debug_img, origin_px, w2p_scaled(0.0, 0.4),
                                (0, 255, 0), 3, tipLength=0.2)

                sequence = [(0.0, 0.0)] + cylinders
                for i in range(len(sequence) - 1):
                    pt1 = w2p_scaled(*sequence[i])
                    pt2 = w2p_scaled(*sequence[i + 1])
                    cv2.arrowedLine(debug_img, pt1, pt2, (0, 200, 255), 2,
                                    tipLength=0.04)

                for i, (wx, wy) in enumerate(cylinders):
                    px = w2p_scaled(wx, wy)
                    cv2.circle(debug_img, px, 10, (0, 0, 255), -1)
                    cv2.putText(debug_img, f'{i}:({wx:.2f},{wy:.2f})',
                                (px[0] + 8, px[1] - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

                debug_path = os.path.expanduser('~/Desktop/path_debug.jpg')
                ok = cv2.imwrite(debug_path, debug_img)
                self.get_logger().info(
                    f'[DEBUG] Waypoint map saved → {debug_path} (ok={ok})')
        except Exception as e:
            self.get_logger().warn(f'[DEBUG] Exception generating debug image: {e}')

        return list(cylinders)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def scan_callback(self, msg):
        self.latest_scan = msg
        inc     = msg.angle_increment
        arc_r   = math.radians(FRONT_ARC_DEG)
        side_r  = math.radians(90)
        front_i = int(round((LIDAR_OFFSET_RAD - msg.angle_min) / inc))
        half_a  = int(round(arc_r  / inc))
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

        left_i  = front_i + int(round(math.radians(90) / inc))
        left_hw = int(round(math.radians(PERIMETER_SIDE_ARC) / inc))
        self.nearest_left_side = arc_min(left_i - left_hw, left_i + left_hw)

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.current_x = pos.x
        self.current_y = pos.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def image_callback(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        self.latest_image = img
        # Detection is evaluated only inside CUBE_FINDING, but we let the
        # flag be set here so the control loop sees it immediately.
        if self.state != 'CUBE_FINDING':
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
        diff = target - current
        while diff >  math.pi: diff -= 2 * math.pi
        while diff < -math.pi: diff += 2 * math.pi
        return diff

    def _distance_to(self, tx, ty):
        return math.sqrt((tx - self.current_x) ** 2 + (ty - self.current_y) ** 2)

    def _estimate_cube_distance(self):
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
            self.get_logger().warn('cv2.imwrite FAILED — snapshot not saved')

    # ── Perimeter following ───────────────────────────────────────────────────

    # ── Perimeter following ───────────────────────────────────────────────────

    def _perimeter_steer(self, target_x, target_y):
        MAX_TRACKING_DIST = PERIMETER_WALL_DIST * 1.5

        # 1. Obstacle Avoidance (Highest Priority)
        if self.nearest_front <= AVOID_DISTANCE:
            if self.nearest_left_side <= MAX_TRACKING_DIST:
                avoid_dir = -1.0
                avoid_reason = "tracking wall -> forcing right"
            else:
                if self.nearest_left >= self.nearest_right:
                    avoid_dir = 1.0
                    avoid_reason = "open space -> left more open"
                else:
                    avoid_dir = -1.0
                    avoid_reason = "open space -> right more open"
            self.get_logger().info(
                f'[AVOID] front={self.nearest_front:.2f} m  '
                f'turning {"left" if avoid_dir > 0 else "right"} ({avoid_reason})')
            return 0.0, avoid_dir * AVOID_TURN_SPEED

        # 2. RIGHT-WALL GUARD (new) ──────────────────────────────────────────
        RIGHT_SAFETY_DIST = 0.6
        if self.nearest_right < RIGHT_SAFETY_DIST:
            right_push = PERIMETER_KP * (RIGHT_SAFETY_DIST - self.nearest_right)
            angular = float(np.clip(right_push, 0.0, AVOID_TURN_SPEED))
            self.get_logger().info(
                f'[RIGHT-GUARD] right={self.nearest_right:.2f} m  pushing left ω={angular:+.2f}')
            return FORWARD_SPEED * 0.7, angular
        # ────────────────────────────────────────────────────────────────────

        # 3. Calculate Heading to Waypoint
        target_angle = math.atan2(
            target_y - self.current_y,
            target_x - self.current_x)
        heading_err = self._angle_diff(target_angle, self.current_yaw)

        # 4. Dynamic Navigation Modes
        if self.nearest_left_side > MAX_TRACKING_DIST:
            nav_mode = "OPEN"
            wall_correction = 0.0
            heading_bias = 0.8 * heading_err
            forward_speed = FORWARD_SPEED
        else:
            nav_mode = "WALL"
            wall_err = PERIMETER_WALL_DIST - self.nearest_left_side
            wall_correction = PERIMETER_KP * wall_err
            heading_bias = 0.3 * heading_err
            forward_speed = FORWARD_SPEED

        angular = float(np.clip(wall_correction + heading_bias,
                                -AVOID_TURN_SPEED, AVOID_TURN_SPEED))
        self.get_logger().info(
            f'[PERIM-{nav_mode}] left_wall={self.nearest_left_side:.2f} m  '
            f'hdg_err={math.degrees(heading_err):+.1f}°  '
            f'ω={angular:+.2f}')
        return forward_speed, angular


    # ── 360° spin helpers ─────────────────────────────────────────────────────

    def _start_spin(self):
        """Initialise bookkeeping for a fresh 360° rotation."""
        self.spin_start_yaw   = self.current_yaw
        self.spin_last_yaw    = self.current_yaw
        self.spin_accumulated = 0.0
        self.cube_detected    = False   # clear stale detections from travel

        # Choose spin direction based on which side has more open space.
        # Positive → counter-clockwise (left), negative → clockwise (right).
        if self.nearest_left >= self.nearest_right:
            self.spin_dir = 1.0
            dir_label = 'CCW (left more open)'
        else:
            self.spin_dir = -1.0
            dir_label = 'CW (right more open)'

        self.get_logger().info(
            f'[SPIN] Starting 360° scan at '
            f'({self.current_x:.2f}, {self.current_y:.2f})  '
            f'yaw₀={math.degrees(self.current_yaw):.1f}°  dir={dir_label}')

    def _update_spin(self):
        """
        Accumulate rotation and return True once a full circle is complete.
        Uses incremental deltas to handle the ±π wrap-around cleanly.
        """
        delta = abs(self._angle_diff(self.current_yaw, self.spin_last_yaw))
        self.spin_accumulated += delta
        self.spin_last_yaw     = self.current_yaw
        return self.spin_accumulated >= SPIN_FULL_CIRCLE

    # ── State machine ─────────────────────────────────────────────────────────

    def control_loop(self):

        # ── Global time limit (only while still searching) ───────────────────
        if self.state in ('WALL_FOLLOWING', 'CUBE_FINDING'):
            if time.time() - self.start_time >= TIME_LIMIT_S:
                self.get_logger().warn('Time limit reached — transitioning to RETURNING')
                self.stop()
                self.state = 'RETURNING'
                return
            
        # ── Position history (only during moving states) ──────────────────────
        if self.state in ('WALL_FOLLOWING', 'RETURNING'):
            self._record_position()

        # ── WALL_FOLLOWING ───────────────────────────────────────────────────
        if self.state == 'WALL_FOLLOWING':

            if self._handle_stuck():
                return

            if self.waypoint_index >= len(self.waypoints):
                self.get_logger().warn(
                    'All waypoints visited, cube not found — transitioning to RETURNING')
                self.stop()
                self.state = 'RETURNING'
                return

            target_x, target_y = self.waypoints[self.waypoint_index]
            dist = self._distance_to(target_x, target_y)

            if dist < WAYPOINT_TOLERANCE:
                # Waypoint reached — switch to 360° scan
                self.stop()
                self.get_logger().info(
                    f'Waypoint {self.waypoint_index} reached '
                    f'({target_x:.2f}, {target_y:.2f}) — starting cube scan')
                self._start_spin()
                self.state = 'CUBE_FINDING'
                return

            linear, angular = self._perimeter_steer(target_x, target_y)
            self._publish_twist(linear, angular)

            self.get_logger().info(
                f'[WALL_FOLLOWING] → WP{self.waypoint_index} '
                f'({target_x:.2f},{target_y:.2f}) '
                f'dist={dist:.2f} m  '
                f'pos=({self.current_x:.2f},{self.current_y:.2f})')

        # ── CUBE_FINDING ─────────────────────────────────────────────────────
        elif self.state == 'CUBE_FINDING':
            if self.cube_detected:
                self.stop()
                self.get_logger().info(
                    f'[CUBE_FINDING] Red cube detected — transitioning to REPORTING')
                self.state = 'REPORTING'
                return

            # ── Nudging forward 20cm before respinning ───────────────────────
            if self.nudging:
                dist_nudged = math.sqrt(
                    (self.current_x - self.nudge_start_x) ** 2 +
                    (self.current_y - self.nudge_start_y) ** 2)
                self.get_logger().info(
                    f'[NUDGE] moved={dist_nudged:.2f} m / {CUBE_SEARCH_STEP_M:.2f} m')

                if dist_nudged >= CUBE_SEARCH_STEP_M:
                    # Nudge complete — start a fresh spin
                    self.stop()
                    self.nudging    = False
                    self.spin_count = 0
                    self._start_spin()
                    self.get_logger().info('[NUDGE] Complete — starting fresh spin')
                else:
                    # Keep nudging, but still respect obstacle avoidance
                    if self.nearest_front <= AVOID_DISTANCE:
                        self.stop()
                        self.get_logger().warn(
                            '[NUDGE] Obstacle during nudge — aborting nudge, respinning')
                        self.nudging    = False
                        self.spin_count = 0
                        self._start_spin()
                    else:
                        self._publish_twist(FORWARD_SPEED, 0.0)
                return

            # ── Spinning ─────────────────────────────────────────────────────
            self._publish_twist(0.0, self.spin_dir * SPIN_SPEED)
            spin_done = self._update_spin()
            self.get_logger().info(
                f'[CUBE_FINDING] spin={self.spin_count+1}/{CUBE_SEARCH_MAX_SPINS}  '
                f'rotated={math.degrees(self.spin_accumulated):.1f}°  '
                f'yaw={math.degrees(self.current_yaw):.1f}°')

            if spin_done:
                self.stop()
                self.spin_count += 1
                self.get_logger().info(
                    f'[CUBE_FINDING] 360° complete ({self.spin_count}/{CUBE_SEARCH_MAX_SPINS})')

                if self.spin_count >= CUBE_SEARCH_MAX_SPINS:
                    # Enough spins — nudge forward and try again
                    self.get_logger().info(
                        f'[CUBE_FINDING] {CUBE_SEARCH_MAX_SPINS} spins done, no cube — '
                        f'nudging {CUBE_SEARCH_STEP_M:.2f} m forward')
                    self.nudging       = True
                    self.nudge_start_x = self.current_x
                    self.nudge_start_y = self.current_y
                else:
                    # Do another spin in the opposite direction for coverage
                    self.spin_dir = -self.spin_dir
                    self._start_spin()
                    self.get_logger().info(
                        f'[CUBE_FINDING] Starting spin {self.spin_count+1} '
                        f'({"CCW" if self.spin_dir > 0 else "CW"})')

        # ── REPORTING ────────────────────────────────────────────────────────
        elif self.state == 'REPORTING':
            self.stop()

            self.cube_x = self.current_x
            self.cube_y = self.current_y

            self.get_logger().info(
                f'Cube position logged at odometry: x={self.cube_x:.3f} m  '
                f'y={self.cube_y:.3f} m  '
                f'(yaw={math.degrees(self.current_yaw):.1f}°)')

            self._save_snapshot()
            self.state = 'RETURNING'

        # ── RETURNING ────────────────────────────────────────────────────────
        elif self.state == 'RETURNING':

            if self._handle_stuck():
                return

            dist = self._distance_to(0.0, 0.0)

            if dist < RETURN_TOLERANCE:
                self.stop()
                self.return_x = self.current_x
                self.return_y = self.current_y
                self.get_logger().info(
                    f'Origin reached — x={self.current_x:.3f} y={self.current_y:.3f}')
                self.state = 'DONE'
                return

            linear, angular = self._perimeter_steer(0.0, 0.0)
            self._publish_twist(linear, angular)

            self.get_logger().info(
                f'[RETURNING] dist={dist:.2f} m  '
                f'pos=({self.current_x:.2f},{self.current_y:.2f})')

        # ── DONE ─────────────────────────────────────────────────────────────
        elif self.state == 'DONE':
            self.stop()
            total_time = time.time() - self.start_time
            self.get_logger().info('=' * 50)
            self.get_logger().info('AUTONOMOUS RUN COMPLETE')
            self.get_logger().info(
                f'Cube position  : ({self.cube_x:.3f}, {self.cube_y:.3f}) m'
                if self.cube_x is not None
                else 'Cube position  : NOT DETECTED')
            self.get_logger().info(
                f'Return position: ({self.return_x:.3f}, {self.return_y:.3f}) m')
            self.get_logger().info(f'Total run time : {total_time:.1f} s')
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