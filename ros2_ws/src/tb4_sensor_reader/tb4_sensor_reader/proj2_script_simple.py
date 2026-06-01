#!/usr/bin/env python3
"""
Autonomous Cube Finder — COMPSYS732 SIMPLE
Phase 2 autonomous search node (Harsher Turning Radius) with CLI Teleop Override.

States: WALL_FOLLOWING → CUBE_FINDING → REPORTING → TURNING_180 → RETURNING → DONE
"""

import rclpy, cv2, math, os, time, sys
import threading, termios, tty, select
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, CompressedImage
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

# ── Robot namespace ────────────────────────────────────────────────────────────
NAMESPACE = '/T10'

# ── Motion parameters (Tuned for harsh/sharp turning radius) ───────────────────
FORWARD_SPEED    = 0.15      # m/s — base waypoint navigation speed
TURN_SPEED       = 0.7       # rad/s — increased for very sharp goal adjustments
AVOID_TURN_SPEED = 0.7       # rad/s — increased for rapid obstacle clearing

# ── Tolerances ─────────────────────────────────────────────────────────────────
WAYPOINT_TOLERANCE  = 0.75   # m — close enough to waypoint to advance
RETURN_TOLERANCE    = 0.15   # m — close enough to origin to stop

# ── Obstacle avoidance & Beam Steering ─────────────────────────────────────────
AVOID_DISTANCE      = 0.4   # m — front cone trigger distance
SIDE_AVOID_DISTANCE = 0.4    # m — diagonal (45°) beam trigger distance
FRONT_ARC_DEG       = 60     # degrees — narrow collision cone directly ahead
LIDAR_OFFSET_DEG    = -90    # degrees — LiDAR mounting offset
LIDAR_OFFSET_RAD    = math.radians(LIDAR_OFFSET_DEG)

# ── Diagonal beam configuration (45° off centre) ──────────────────────────────
BEAM_ANGLE_DEG      = 45     # degrees — offset to the left/right of centre
BEAM_WIDTH_DEG      = 10     # degrees — width of each diagonal beam window

# ── Lateral beam configuration (90° off centre — pure left / pure right) ──────
LATERAL_BEAM_ANGLE_DEG     = 90    # degrees — offset from centre for lateral beams
                                    # Change this to shift both lateral beams simultaneously.
                                    # Positive values push them rearward (>90°);
                                    # Negative values pull them forward (<90°).
LATERAL_BEAM_WIDTH_DEG     = 10    # degrees — arc width of each lateral beam window
LATERAL_AVOID_DISTANCE     = 0.3  # m — trigger threshold for lateral beams
                                    # Independent from SIDE_AVOID_DISTANCE so you can
                                    # tune lateral sensitivity without affecting the 45° beams.

# ── Red cube detection constants ───────────────────────────────────────────────
RED_LOW1   = np.array([0,   150, 100])
RED_HIGH1  = np.array([8,   255, 255])
RED_LOW2   = np.array([172, 150, 100])
RED_HIGH2  = np.array([180, 255, 255])

# ── Central Pillar Settings ────────────────────────────────────────────────────
SLICE_WIDTH = 50             # Width of the central vertical corridor in pixels
MIN_PIXELS  = 1000           # Min red pixels inside the slice to trigger

# ── File Output Paths ──────────────────────────────────────────────────────────
SNAPSHOT_PATH = os.path.expanduser('~/Desktop/detection_snapshot.jpg')
RESULTS_PATH  = os.path.expanduser('~/Desktop/RESULTS.txt')

# ── LiDAR cube distance estimation ────────────────────────────────────────────
CUBE_FORWARD_ARC_DEG = 3     
LIDAR_FALLBACK_DIST  = 0.30  

# ── 360° spin (CUBE_FINDING state) ────────────────────────────────────────────
SPIN_SPEED       = 1       
SPIN_FULL_CIRCLE = 2 * math.pi  
CUBE_SEARCH_STEP_M   = 0.20   
CUBE_SEARCH_MAX_SPINS = 1     

# ── Map paths ─────────────────────────────────────────────────────────────────
PGM_MAP_PATH = os.path.expanduser('~/Desktop/lab_map_c_fri.pgm')
PGM_MAP_YAML = os.path.expanduser('~/Desktop/lab_map_c_fri.yaml')

# ── Manual waypoint fallback ───────────────────────────────────────────────────
MANUAL_WAYPOINTS = [
    (-2.3, -2.3),
    (1.5, 0.5),
    (1.5, 1.0),
]

# ── Keyboard Listener Thread ───────────────────────────────────────────────────
class KeyboardThread(threading.Thread):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.daemon = True
        self.running = True
        self.old_settings = termios.tcgetattr(sys.stdin)
        
    def run(self):
        tty.setcbreak(sys.stdin.fileno())
        try:
            while self.running:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key == '\x03':  # Ctrl-C forces exit
                        break
                    self.node.process_key(key)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


# ── Cylinder feature extraction ────────────────────────────────────────────────
def extract_cylinder_waypoints(pgm_path, yaml_path):
    try:
        import yaml
        with open(yaml_path, 'r') as f:
            meta = yaml.safe_load(f)
        resolution = meta.get('resolution')
        origin     = meta.get('origin')
        
        img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if img is None: return None
        h, w = img.shape
        
        _, binary = cv2.threshold(img, 50, 255, cv2.THRESH_BINARY_INV)
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            binary, connectivity=8)
            
        wall_label = 1 + int(np.argmax(
            [stats[i, cv2.CC_STAT_AREA] for i in range(1, num_labels)]))
            
        wall_mask = (labels == wall_label).astype(np.uint8) * 255
        contours, _ = cv2.findContours(wall_mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        border_contour = max(contours, key=cv2.contourArea)

        def is_inside_border(px, py):
            return cv2.pointPolygonTest(border_contour, (float(px), float(py)), False) >= 0

        MIN_BLOB_AREA = 4
        MAX_BLOB_AREA = 30
        waypoints = []

        for i in range(1, num_labels):
            if i == wall_label: continue
            area   = stats[i, cv2.CC_STAT_AREA]
            blob_w = stats[i, cv2.CC_STAT_WIDTH]
            blob_h = stats[i, cv2.CC_STAT_HEIGHT]
            cx, cy = centroids[i]

            if area < MIN_BLOB_AREA or area > MAX_BLOB_AREA: continue
            if blob_w < 2 or blob_h < 2: continue
            aspect_ratio = float(blob_w) / float(blob_h)
            if aspect_ratio < 0.5 or aspect_ratio > 2.0: continue
            if not is_inside_border(cx, cy): continue

            world_x = origin[0] + cx * resolution
            world_y = origin[1] + (h - cy) * resolution
            waypoints.append((round(world_x, 3), round(world_y, 3)))

        # ── Feature map saving (Pulled from WALL_FOLLOWING) ───────────────────
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
        print(f'[EXTRACT] Exception: {e}')
        return None

# ── Main node ──────────────────────────────────────────────────────────────────
class AutonomousSearch(Node):

    def __init__(self):
        super().__init__('autonomous_search')
        self.bridge = CvBridge()

        # ── Publishers & Subscribers ─────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, f'{NAMESPACE}/cmd_vel', 10)
        self.create_subscription(LaserScan, f'{NAMESPACE}/scan', self.scan_callback, 10)
        self.create_subscription(CompressedImage, f'{NAMESPACE}/oakd/rgb/image_raw/compressed', self.image_callback, 10)
        self.create_subscription(Odometry, f'{NAMESPACE}/odom', self.odom_callback, 10)

        # ── Sensor state ─────────────────────────────────────────────────────
        self.current_x              = 0.0
        self.current_y              = 0.0
        self.current_yaw            = 0.0
        self.nearest_front          = float('inf')
        self.beam_left_dist         = float('inf')   # diagonal left  (45°)
        self.beam_right_dist        = float('inf')   # diagonal right (45°)
        self.lateral_left_dist      = float('inf')   # pure left      (90°)
        self.lateral_right_dist     = float('inf')   # pure right     (90°)
        self.latest_scan            = None
        self.latest_image           = None
        self.cube_detected          = False
        self.odom_received          = False

        # ── Detection snapshot pose ───────────────────────────────────────────
        self.detection_robot_x = None
        self.detection_robot_y = None
        self.detection_yaw     = None

        # ── End-of-run bookkeeping ────────────────────────────────────────────
        self.run_end_time  = None
        self.done_printed  = False
        
        # ── Movement / Avoidance state ────────────────────────────────────────
        self.avoid_dir         = 0     
        
        # ── 360° spin bookkeeping ─────────────────────────────────────────────
        self.spin_dir         = 1.0   
        self.spin_count        = 0     
        self.spin_start_yaw   = None
        self.spin_accumulated = 0.0
        self.spin_last_yaw    = None
        self.nudge_start_x     = None  
        self.nudge_start_y     = None
        self.nudging           = False  
        
        # ── 180° turn bookkeeping ─────────────────────────────────────────────
        self.turn_180_last_yaw    = None
        self.turn_180_accumulated = 0.0

        # ── Teleoperation state ───────────────────────────────────────────────
        self.teleop_mode       = False
        self.teleop_linear     = 0.0
        self.teleop_angular    = 0.0
        self.last_key_time     = 0.0

        # ── Waypoints ────────────────────────────────────────────────────────
        self.waypoints      = self._load_waypoints()
        self.waypoint_index = 0

        # ── State machine ─────────────────────────────────────────────────────
        resume_mode = None
        for arg in sys.argv[1:]:
            if arg.startswith('--resume='):
                resume_mode = arg.split('=', 1)[1].upper()

        if resume_mode == 'RETURN':
            initial_state = 'RETURNING'
        elif resume_mode == 'SEARCH':
            initial_state = 'WALL_FOLLOWING'
        elif resume_mode == 'CUBE':
            initial_state = 'CUBE_FINDING'
        else:
            initial_state = 'WALL_FOLLOWING'

        self.start_time = time.time()
        self.cube_x, self.cube_y, self.return_x, self.return_y = None, None, None, None
        
        self.state = None 
        self.set_state(initial_state)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Autonomous search started (Harsh Turning Active). Press keys in terminal to interrupt.')

    def process_key(self, key):
        """Handle incoming keyboard presses from the background thread."""
        self.last_key_time = time.time()
        
        if key == 't':
            self.teleop_mode = not self.teleop_mode
            mode_str = "ON" if self.teleop_mode else "OFF"
            self.get_logger().info(f'[TELEOP] Teleoperation mode: {mode_str}')
            self.stop()
            self.teleop_linear = 0.0
            self.teleop_angular = 0.0
        elif key == 'r':
            self.get_logger().info('[KEYBOARD] Force switching to RETURNING state')
            self.teleop_mode = False
            self.set_state('RETURNING')
        elif key == 's':
            self.get_logger().info('[KEYBOARD] Force switching to WALL_FOLLOWING (Search) state')
            self.teleop_mode = False
            self.set_state('WALL_FOLLOWING')
        elif key == 'c':
            self.get_logger().info('[KEYBOARD] Force switching to CUBE_FINDING state')
            self.teleop_mode = False
            self._start_spin()
            self.set_state('CUBE_FINDING')
        
        if self.teleop_mode:
            if key == 'i':
                self.teleop_linear = FORWARD_SPEED
                self.teleop_angular = 0.0
            elif key == 'j':
                self.teleop_linear = 0.0
                self.teleop_angular = TURN_SPEED
            elif key == 'l':
                self.teleop_linear = 0.0
                self.teleop_angular = -TURN_SPEED

    def set_state(self, new_state):
        if self.state == new_state:
            return
        if self.state is not None:
            self.get_logger().info(f'[STATE] Exiting state: {self.state}')
        self.state = new_state
        self.get_logger().info(f'[STATE] Entering state: {self.state}')

        if new_state == 'DONE' and self.run_end_time is None:
            self.run_end_time = time.time()

    def _load_waypoints(self):
        if PGM_MAP_PATH is None or not os.path.exists(PGM_MAP_PATH):
            return list(MANUAL_WAYPOINTS)
        cylinders = extract_cylinder_waypoints(PGM_MAP_PATH, PGM_MAP_YAML)
        if not cylinders:
            return list(MANUAL_WAYPOINTS)
        return list(cylinders)

    def scan_callback(self, msg):
        self.latest_scan = msg
        inc = msg.angle_increment
        n   = len(msg.ranges)

        front_i = int(round((LIDAR_OFFSET_RAD - msg.angle_min) / inc))

        # ── Arc index helpers ────────────────────────────────────────────────
        front_half_a       = int(round(math.radians(FRONT_ARC_DEG)          / 2.0 / inc))
        beam_half_w        = int(round(math.radians(BEAM_WIDTH_DEG)         / 2.0 / inc))
        beam_offset        = int(round(math.radians(BEAM_ANGLE_DEG)         / inc))
        lateral_half_w     = int(round(math.radians(LATERAL_BEAM_WIDTH_DEG) / 2.0 / inc))
        lateral_offset     = int(round(math.radians(LATERAL_BEAM_ANGLE_DEG) / inc))

        # Diagonal beams (±45°)
        left_beam_i        = front_i + beam_offset
        right_beam_i       = front_i - beam_offset

        # Lateral beams (±90°, or whatever LATERAL_BEAM_ANGLE_DEG is set to)
        lateral_left_i     = front_i + lateral_offset
        lateral_right_i    = front_i - lateral_offset

        def arc_min(center_idx, half_angle_idx):
            lo = max(0, center_idx - half_angle_idx)
            hi = min(n - 1, center_idx + half_angle_idx)
            vals = [r for r in msg.ranges[lo:hi+1]
                    if msg.range_min < r < msg.range_max
                    and not math.isnan(r) and not math.isinf(r)]
            return min(vals) if vals else float('inf')

        self.nearest_front      = arc_min(front_i,        front_half_a)
        self.beam_left_dist     = arc_min(left_beam_i,    beam_half_w)
        self.beam_right_dist    = arc_min(right_beam_i,   beam_half_w)
        self.lateral_left_dist  = arc_min(lateral_left_i, lateral_half_w)
        self.lateral_right_dist = arc_min(lateral_right_i, lateral_half_w)

    def odom_callback(self, msg):
        self.odom_received = True
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
        if self.state != 'CUBE_FINDING': return

        hsv  = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(cv2.inRange(hsv, RED_LOW1, RED_HIGH1), cv2.inRange(hsv, RED_LOW2, RED_HIGH2))
        
        h, w = mask.shape[:2]
        mid_x = w // 2
        half_w = SLICE_WIDTH // 2
        
        slice_mask = mask[:, max(0, mid_x - half_w):min(w, mid_x + half_w)]
        if cv2.countNonZero(slice_mask) >= MIN_PIXELS:
            self.cube_detected = True

    def stop(self):
        self.cmd_pub.publish(Twist())

    def _publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        self.cmd_pub.publish(msg)

    def _angle_diff(self, target, current):
        diff = target - current
        while diff >  math.pi: diff -= 2 * math.pi
        while diff < -math.pi: diff += 2 * math.pi
        return diff

    def _distance_to(self, tx, ty):
        return math.sqrt((tx - self.current_x) ** 2 + (ty - self.current_y) ** 2)

    def _estimate_cube_distance(self):
        if self.latest_scan is None: return LIDAR_FALLBACK_DIST
        msg = self.latest_scan
        inc, arc_r = msg.angle_increment, math.radians(CUBE_FORWARD_ARC_DEG)
        front_i = int(round((LIDAR_OFFSET_RAD - msg.angle_min) / inc))
        half_a, n = int(round(arc_r / inc)), len(msg.ranges)
        lo, hi = max(0, front_i - half_a), min(n - 1, front_i + half_a)
        valid = [r for r in msg.ranges[lo:hi+1] if msg.range_min < r < msg.range_max and not math.isnan(r) and not math.isinf(r)]
        return min(valid) if valid else LIDAR_FALLBACK_DIST

    def _save_snapshot(self):
        if self.latest_image is not None: cv2.imwrite(SNAPSHOT_PATH, self.latest_image)

    def _save_results_text(self, rx, ry, yaw_rad, distance, cx, cy):
        """
        Save detection results with all intermediate values so the cube position
        calculation can be verified manually.

        Cube position formula:
            cube_x = robot_x + distance * cos(yaw_rad)
            cube_y = robot_y + distance * sin(yaw_rad)
        """
        yaw_deg = math.degrees(yaw_rad)
        try:
            with open(RESULTS_PATH, 'w') as f:
                f.write("=" * 50 + "\n")
                f.write("  AUTONOMOUS CUBE FINDER — RESULTS\n")
                f.write("=" * 50 + "\n\n")

                f.write("── Robot pose at detection ──────────────────────\n")
                f.write(f"  ROBOT_X          : {rx:.6f} m\n")
                f.write(f"  ROBOT_Y          : {ry:.6f} m\n\n")

                f.write("── LiDAR offset measurement ─────────────────────\n")
                f.write(f"  DETECTION_YAW    : {yaw_rad:.6f} rad  ({yaw_deg:.4f} deg)\n")
                f.write(f"  OFFSET_DISTANCE  : {distance:.6f} m\n\n")

                f.write("── Cube position calculation (do it yourself) ───\n")
                f.write(f"  cube_x = robot_x + distance * cos(yaw_rad)\n")
                f.write(f"         = {rx:.6f} + {distance:.6f} * cos({yaw_rad:.6f})\n")
                f.write(f"         = {rx:.6f} + {distance:.6f} * {math.cos(yaw_rad):.6f}\n")
                f.write(f"         = {cx:.6f} m\n\n")
                f.write(f"  cube_y = robot_y + distance * sin(yaw_rad)\n")
                f.write(f"         = {ry:.6f} + {distance:.6f} * sin({yaw_rad:.6f})\n")
                f.write(f"         = {ry:.6f} + {distance:.6f} * {math.sin(yaw_rad):.6f}\n")
                f.write(f"         = {cy:.6f} m\n\n")

                f.write("── Reported cube position ───────────────────────\n")
                f.write(f"  CUBE_X           : {cx:.6f} m\n")
                f.write(f"  CUBE_Y           : {cy:.6f} m\n")
                f.write("=" * 50 + "\n")
        except Exception:
            pass

    def _simple_steer(self, target_x, target_y):
        target_angle = math.atan2(target_y - self.current_y, target_x - self.current_x)
        heading_err  = self._angle_diff(target_angle, self.current_yaw)

        front_blocked         = self.nearest_front        <= AVOID_DISTANCE
        left_blocked          = self.beam_left_dist       <= SIDE_AVOID_DISTANCE
        right_blocked         = self.beam_right_dist      <= SIDE_AVOID_DISTANCE
        lateral_left_blocked  = self.lateral_left_dist    <= LATERAL_AVOID_DISTANCE
        lateral_right_blocked = self.lateral_right_dist   <= LATERAL_AVOID_DISTANCE

        # ── Priority 1: front 60° cone ───────────────────────────────────────
        # Commit to a direction on first trigger; compare side beams to pick the
        # more open side.  Hold that direction until the cone is clear again.
        if front_blocked:
            if self.avoid_dir == 0:
                if self.beam_left_dist >= self.beam_right_dist:
                    self.avoid_dir = 1.0   # left is more open — pivot left
                else:
                    self.avoid_dir = -1.0  # right is more open — pivot right
                self.get_logger().info(
                    f'[AVOID] Front blocked ({self.nearest_front:.2f}m). '
                    f'L-beam: {self.beam_left_dist:.2f}m  R-beam: {self.beam_right_dist:.2f}m  '
                    f'→ pivoting {"LEFT" if self.avoid_dir > 0 else "RIGHT"}'
                )
            return 0.0, self.avoid_dir * AVOID_TURN_SPEED

        # ── Priority 2: diagonal side beams (45°) ────────────────────────────
        # If only the left beam fires, the left side is tight — turn right.
        # If only the right beam fires, the right side is tight — turn left.
        # If both fire, fall through; the front cone would have caught it first.
        elif left_blocked and not right_blocked:
            if self.avoid_dir != -1.0:
                self.avoid_dir = -1.0
                self.get_logger().info(
                    f'[AVOID] Diagonal-left beam triggered ({self.beam_left_dist:.2f}m) → turning RIGHT'
                )
            return 0.0, self.avoid_dir * AVOID_TURN_SPEED

        elif right_blocked and not left_blocked:
            if self.avoid_dir != 1.0:
                self.avoid_dir = 1.0
                self.get_logger().info(
                    f'[AVOID] Diagonal-right beam triggered ({self.beam_right_dist:.2f}m) → turning LEFT'
                )
            return 0.0, self.avoid_dir * AVOID_TURN_SPEED

        # ── Priority 3: lateral beams (90°) ──────────────────────────────────
        # These guard the flanks while the robot is moving forward.
        # A blocked lateral-left means the left wall is very close — steer right.
        # A blocked lateral-right means the right wall is very close — steer left.
        # If both lateral beams fire simultaneously (narrow corridor), prefer
        # whichever side has more clearance, or hold the current avoid direction.
        elif lateral_left_blocked and not lateral_right_blocked:
            if self.avoid_dir != -1.0:
                self.avoid_dir = -1.0
                self.get_logger().info(
                    f'[AVOID] Lateral-left beam triggered ({self.lateral_left_dist:.2f}m) → steering RIGHT'
                )
            # Allow forward motion at reduced speed while correcting
            return FORWARD_SPEED * 0.5, self.avoid_dir * AVOID_TURN_SPEED

        elif lateral_right_blocked and not lateral_left_blocked:
            if self.avoid_dir != 1.0:
                self.avoid_dir = 1.0
                self.get_logger().info(
                    f'[AVOID] Lateral-right beam triggered ({self.lateral_right_dist:.2f}m) → steering LEFT'
                )
            return FORWARD_SPEED * 0.5, self.avoid_dir * AVOID_TURN_SPEED

        elif lateral_left_blocked and lateral_right_blocked:
            # Both flanks tight — keep whichever avoid direction is already set,
            # or default to steering away from the closer side.
            if self.avoid_dir == 0:
                self.avoid_dir = 1.0 if self.lateral_left_dist <= self.lateral_right_dist else -1.0
                self.get_logger().info(
                    f'[AVOID] Both lateral beams triggered '
                    f'(L:{self.lateral_left_dist:.2f}m R:{self.lateral_right_dist:.2f}m) '
                    f'→ steering {"LEFT" if self.avoid_dir > 0 else "RIGHT"}'
                )
            return FORWARD_SPEED * 0.5, self.avoid_dir * AVOID_TURN_SPEED

        # ── Priority 4: all clear — resume goal tracking ─────────────────────
        else:
            if self.avoid_dir != 0:
                if abs(heading_err) < 0.2:
                    self.get_logger().info('[AVOID] All beams clear. Resuming goal tracking.')
                    self.avoid_dir = 0
                    return FORWARD_SPEED, 0.0
                else:
                    return FORWARD_SPEED * 0.4, float(np.clip(heading_err * 3.0, -AVOID_TURN_SPEED, AVOID_TURN_SPEED))
            else:
                angular      = float(np.clip(heading_err * 3.0, -TURN_SPEED, TURN_SPEED))
                speed_factor = max(0.2, 1.0 - (abs(heading_err) / (math.pi / 2)))
                return FORWARD_SPEED * speed_factor, angular

    def _start_spin(self):
        self.spin_start_yaw, self.spin_last_yaw = self.current_yaw, self.current_yaw
        self.spin_accumulated, self.cube_detected = 0.0, False
        self.spin_dir = 1.0 if self.beam_left_dist >= self.beam_right_dist else -1.0

    def _update_spin(self):
        delta = abs(self._angle_diff(self.current_yaw, self.spin_last_yaw))
        self.spin_accumulated += delta
        self.spin_last_yaw = self.current_yaw
        return self.spin_accumulated >= SPIN_FULL_CIRCLE

    def control_loop(self):
        if not self.odom_received: return

        # ── TELEOPERATION OVERRIDE ───────────────────────────────────────────
        if self.teleop_mode:
            if time.time() - self.last_key_time > 0.2:
                self.teleop_linear = 0.0
                self.teleop_angular = 0.0
            self._publish_twist(self.teleop_linear, self.teleop_angular)
            return

        # ── WALL_FOLLOWING ───────────────────────────────────────────────────
        if self.state == 'WALL_FOLLOWING':
            if self.waypoint_index >= len(self.waypoints):
                self.stop()
                self.set_state('RETURNING')
                return

            target_x, target_y = self.waypoints[self.waypoint_index]
            dist = self._distance_to(target_x, target_y)

            if dist < WAYPOINT_TOLERANCE:
                self.stop()
                self.avoid_dir = 0 
                self.get_logger().info(f'Waypoint {self.waypoint_index} reached — starting cube scan')
                self._start_spin()
                self.set_state('CUBE_FINDING')
                return

            linear, angular = self._simple_steer(target_x, target_y)
            self._publish_twist(linear, angular)

        # ── CUBE_FINDING ─────────────────────────────────────────────────────
        elif self.state == 'CUBE_FINDING':
            if self.spin_start_yaw is None:
                self._start_spin()

            if self.cube_detected:
                self.stop()
                self.avoid_dir = 0
                self.set_state('REPORTING')
                return

            if self.nudging:
                dist_nudged = math.sqrt((self.current_x - self.nudge_start_x) ** 2 + (self.current_y - self.nudge_start_y) ** 2)
                if dist_nudged >= CUBE_SEARCH_STEP_M or self.nearest_front <= AVOID_DISTANCE:
                    self.stop()
                    self.nudging, self.spin_count = False, 0
                    self._start_spin()
                else:
                    self._publish_twist(FORWARD_SPEED, 0.0)
                return

            self._publish_twist(0.0, self.spin_dir * SPIN_SPEED)
            if self._update_spin():
                self.stop()
                self.spin_count += 1
                if self.spin_count >= CUBE_SEARCH_MAX_SPINS:
                    self.nudging = True
                    self.nudge_start_x, self.nudge_start_y = self.current_x, self.current_y
                else:
                    self.spin_dir = -self.spin_dir
                    self._start_spin()

        # ── REPORTING ────────────────────────────────────────────────────────
        elif self.state == 'REPORTING':
            self.stop()

            self.detection_robot_x = self.current_x
            self.detection_robot_y = self.current_y
            self.detection_yaw     = self.current_yaw

            cube_dist  = self._estimate_cube_distance()
            self.cube_x = self.detection_robot_x + cube_dist * math.cos(self.detection_yaw)
            self.cube_y = self.detection_robot_y + cube_dist * math.sin(self.detection_yaw)

            self._save_results_text(
                self.detection_robot_x, self.detection_robot_y,
                self.detection_yaw, cube_dist,
                self.cube_x, self.cube_y
            )
            self._save_snapshot()
            
            self.turn_180_last_yaw = self.current_yaw
            self.turn_180_accumulated = 0.0
            self.set_state('TURNING_180')

        # ── TURNING_180 ──────────────────────────────────────────────────────
        elif self.state == 'TURNING_180':
            delta = abs(self._angle_diff(self.current_yaw, self.turn_180_last_yaw))
            self.turn_180_accumulated += delta
            self.turn_180_last_yaw = self.current_yaw
            
            if self.turn_180_accumulated >= math.pi:
                self.stop()
                self.set_state('RETURNING')
            else:
                self._publish_twist(0.0, TURN_SPEED)

        # ── RETURNING ────────────────────────────────────────────────────────
        elif self.state == 'RETURNING':
            dist = self._distance_to(0.0, 0.0)
            if dist < RETURN_TOLERANCE:
                self.stop()
                self.return_x, self.return_y = self.current_x, self.current_y
                self.set_state('DONE')
                return

            linear, angular = self._simple_steer(0.0, 0.0)
            self._publish_twist(linear, angular)

        # ── DONE ─────────────────────────────────────────────────────────────
        elif self.state == 'DONE':
            self.stop()

            if not self.done_printed:
                self.done_printed = True

                elapsed = (self.run_end_time or time.time()) - self.start_time
                mins, secs = divmod(elapsed, 60)

                sep = '=' * 54
                self.get_logger().info(sep)
                self.get_logger().info('  AUTONOMOUS RUN COMPLETE')
                self.get_logger().info(sep)
                if self.cube_x is not None:
                    self.get_logger().info(
                        f'  Cube detected at      : ({self.cube_x:.4f}, {self.cube_y:.4f}) m')
                else:
                    self.get_logger().info('  Cube detected at      : NOT FOUND')
                if self.detection_robot_x is not None:
                    self.get_logger().info(
                        f'  Robot pos at detection: ({self.detection_robot_x:.4f}, {self.detection_robot_y:.4f}) m')
                    self.get_logger().info(
                        f'  Heading at detection  : {math.degrees(self.detection_yaw):.2f} deg  ({self.detection_yaw:.4f} rad)')
                else:
                    self.get_logger().info('  Robot pos at detection: N/A (cube not found)')
                if self.return_x is not None:
                    self.get_logger().info(
                        f'  Return position       : ({self.return_x:.4f}, {self.return_y:.4f}) m')
                else:
                    self.get_logger().info('  Return position       : N/A')
                self.get_logger().info(
                    f'  Total run time        : {int(mins)}m {secs:.1f}s  ({elapsed:.1f} s)')
                self.get_logger().info(sep)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousSearch()
    
    kb_thread = KeyboardThread(node)
    kb_thread.start()
    
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally:
        kb_thread.running = False
        kb_thread.join(timeout=0.5)
        cv2.destroyAllWindows()
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()