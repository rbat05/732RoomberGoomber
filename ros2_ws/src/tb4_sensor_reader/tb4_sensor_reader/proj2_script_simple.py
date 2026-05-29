#!/usr/bin/env python3
"""
Autonomous Cube Finder — COMPSYS732
Phase 2 autonomous search node (Harsher Turning Radius).

States: WALL_FOLLOWING → CUBE_FINDING → REPORTING → TURNING_180 → RETURNING → DONE
"""

import rclpy, cv2, math, os, time, sys
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, CompressedImage
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

# ── Robot namespace ────────────────────────────────────────────────────────────
NAMESPACE = '/T19'

# ── Motion parameters (Tuned for harsh/sharp turning radius) ───────────────────
FORWARD_SPEED    = 0.15      # m/s — base waypoint navigation speed
TURN_SPEED       = 1.5       # rad/s — increased for very sharp goal adjustments
AVOID_TURN_SPEED = 1.2       # rad/s — increased for rapid obstacle clearing

# ── Tolerances ─────────────────────────────────────────────────────────────────
WAYPOINT_TOLERANCE  = 0.75   # m — close enough to waypoint to advance
RETURN_TOLERANCE    = 0.15   # m — close enough to origin to stop

# ── Obstacle avoidance & Beam Steering ─────────────────────────────────────────
AVOID_DISTANCE      = 0.3    # m — obstacle trigger distance
FRONT_ARC_DEG       = 140     # degrees — purely for collision/front obstacle detection
LIDAR_OFFSET_DEG    = -90    # degrees — LiDAR mounting offset
LIDAR_OFFSET_RAD    = math.radians(LIDAR_OFFSET_DEG)

# New narrow beam configurations for directional steering choices
BEAM_ANGLE_DEG      = 45     # degrees — offset to the left/right of center for the beams
BEAM_WIDTH_DEG      = 10      # degrees — width of the small beam windows

# ── Time limit ─────────────────────────────────────────────────────────────────
TIME_LIMIT_S = 420.0         # 7 minutes — force RETURNING if cube not found

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

        # ── COPIED VISUALIZATION FROM PROJ2_SCRIPT_WALL_FOLLOWING ───────────────────
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
        # ───────────────────────────────────────────────────────────────────────────

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
        self.current_x         = 0.0
        self.current_y         = 0.0
        self.current_yaw       = 0.0
        self.nearest_front     = float('inf')
        self.beam_left_dist    = float('inf')  # Narrow steering beam (left)
        self.beam_right_dist   = float('inf')  # Narrow steering beam (right)
        self.latest_scan       = None
        self.latest_image      = None
        self.cube_detected     = False
        self.odom_received     = False
        
        # ── Movement / Avoidance state ────────────────────────────────────────
        self.avoid_dir         = 0     # 0 = direct to goal, 1 = left, -1 = right
        
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
        
        self.state = None # Set to None initially so the entry log is clean
        self.set_state(initial_state)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f'Autonomous search started (Harsh Turning Active)')

    def set_state(self, new_state):
        """Helper to centralize state assignments and log exits/entries automatically."""
        if self.state == new_state:
            return
            
        if self.state is not None:
            self.get_logger().info(f'[STATE] Exiting state: {self.state}')
            
        self.state = new_state
        self.get_logger().info(f'[STATE] Entering state: {self.state}')

    def _load_waypoints(self):
        if PGM_MAP_PATH is None or not os.path.exists(PGM_MAP_PATH):
            return list(MANUAL_WAYPOINTS)
        cylinders = extract_cylinder_waypoints(PGM_MAP_PATH, PGM_MAP_YAML)
        if not cylinders:
            return list(MANUAL_WAYPOINTS)

        # ── COPIED PATH MAP VISUALIZATION FROM PROJ2_SCRIPT_WALL_FOLLOWING ──────────
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
        # ───────────────────────────────────────────────────────────────────────────

        return list(cylinders)

    def scan_callback(self, msg):
        self.latest_scan = msg
        inc     = msg.angle_increment
        n       = len(msg.ranges)
        
        # 1. Front Detection Arc index calculations (Used purely for tracking front obstacles)
        front_i = int(round((LIDAR_OFFSET_RAD - msg.angle_min) / inc))
        front_half_a = int(round(math.radians(FRONT_ARC_DEG) / 2.0 / inc))

        # 2. Narrow Decision-Making Beam index calculations
        beam_half_w  = int(round(math.radians(BEAM_WIDTH_DEG) / 2.0 / inc))
        beam_offset  = int(round(math.radians(BEAM_ANGLE_DEG) / inc))
        
        left_beam_i  = front_i + beam_offset
        right_beam_i = front_i - beam_offset

        def arc_min(center_idx, half_angle_idx):
            lo = max(0, center_idx - half_angle_idx)
            hi = min(n - 1, center_idx + half_angle_idx)
            vals = [r for r in msg.ranges[lo:hi+1]
                    if msg.range_min < r < msg.range_max and not math.isnan(r) and not math.isinf(r)]
            return min(vals) if vals else float('inf')

        # Evaluate the front arc and the two small steering beams
        self.nearest_front   = arc_min(front_i, front_half_a)
        self.beam_left_dist  = arc_min(left_beam_i, beam_half_w)
        self.beam_right_dist = arc_min(right_beam_i, beam_half_w)

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

    def _save_results_text(self, rx, ry, offset, cx, cy):
        try:
            with open(RESULTS_PATH, 'w') as f:
                f.write(f"ROBOT_X: {rx:.4f}\nROBOT_Y: {ry:.4f}\nOFFSET_DISTANCE: {offset:.4f}\n")
                f.write(f"REPORTED_CUBE_X: {cx:.4f}\nREPORTED_CUBE_Y: {cy:.4f}\n")
        except Exception: pass

    def _simple_steer(self, target_x, target_y):
        """Direct-to-goal with narrow-beam obstacle verification."""
        target_angle = math.atan2(target_y - self.current_y, target_x - self.current_x)
        heading_err = self._angle_diff(target_angle, self.current_yaw)

        # 1. Obstacle inside front arc -> Switch to evasive rotation using the narrow beams
        if self.nearest_front <= AVOID_DISTANCE:
            if self.avoid_dir == 0:
                # Decide turning direction based on the larger beam reading (clearer side)
                if self.beam_left_dist >= self.beam_right_dist:
                    self.avoid_dir = 1.0  # Turn left
                else:
                    self.avoid_dir = -1.0 # Turn right
                self.get_logger().info(f'[AVOID] Front block. Left beam: {self.beam_left_dist:.2f}m, Right beam: {self.beam_right_dist:.2f}m. Pivoting {"LEFT" if self.avoid_dir > 0 else "RIGHT"}')
            
            return 0.0, self.avoid_dir * AVOID_TURN_SPEED

        # 2. Path directly ahead is clear
        else:
            if self.avoid_dir != 0:
                # Is the goal clear enough to stop tracing?
                if abs(heading_err) < 0.2:
                    self.get_logger().info('[AVOID] Target clear. Resuming goal tracking.')
                    self.avoid_dir = 0
                    return FORWARD_SPEED, 0.0
                else:
                    # Tight contouring arc away from obstacle
                    return FORWARD_SPEED * 0.4, -self.avoid_dir * (AVOID_TURN_SPEED * 0.85)
            
            # 3. Standard track to goal
            else:
                angular = float(np.clip(heading_err * 3.0, -TURN_SPEED, TURN_SPEED))
                speed_factor = max(0.2, 1.0 - (abs(heading_err) / (math.pi / 2)))
                return FORWARD_SPEED * speed_factor, angular

    def _start_spin(self):
        self.spin_start_yaw, self.spin_last_yaw = self.current_yaw, self.current_yaw
        self.spin_accumulated, self.cube_detected = 0.0, False
        # Choose spin direction toward the side with more clearance according to narrow beams
        self.spin_dir = 1.0 if self.beam_left_dist >= self.beam_right_dist else -1.0

    def _update_spin(self):
        delta = abs(self._angle_diff(self.current_yaw, self.spin_last_yaw))
        self.spin_accumulated += delta
        self.spin_last_yaw = self.current_yaw
        return self.spin_accumulated >= SPIN_FULL_CIRCLE

    def control_loop(self):
        if not self.odom_received: return

        # if self.state in ('WALL_FOLLOWING', 'CUBE_FINDING'):
        #     if time.time() - self.start_time >= TIME_LIMIT_S:
        #         self.stop()
        #         self.set_state('RETURNING')
        #         return

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
            cube_dist = self._estimate_cube_distance()
            self.cube_x = self.current_x + cube_dist * math.cos(self.current_yaw)
            self.cube_y = self.current_y + cube_dist * math.sin(self.current_yaw)
            self._save_results_text(self.current_x, self.current_y, cube_dist, self.cube_x, self.cube_y)
            self._save_snapshot()
            
            # Start the 180 degree turn setup
            self.turn_180_last_yaw = self.current_yaw
            self.turn_180_accumulated = 0.0
            self.set_state('TURNING_180')

        # ── TURNING_180 ──────────────────────────────────────────────────────
        elif self.state == 'TURNING_180':
            # Accumulate angular distance
            delta = abs(self._angle_diff(self.current_yaw, self.turn_180_last_yaw))
            self.turn_180_accumulated += delta
            self.turn_180_last_yaw = self.current_yaw
            
            # Complete when 180 degrees (pi radians) is reached
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
            self.get_logger().info('AUTONOMOUS RUN COMPLETE')

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousSearch()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        cv2.destroyAllWindows()
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()