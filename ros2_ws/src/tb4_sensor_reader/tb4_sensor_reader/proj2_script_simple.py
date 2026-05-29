#!/usr/bin/env python3
"""
Autonomous Cube Finder — COMPSYS732
Phase 2 autonomous search node (Harsher Turning Radius).

States: WALL_FOLLOWING → CUBE_FINDING → REPORTING → RETURNING → DONE
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

# ── Motion parameters (Tuned for harsh/sharp turning radius) ───────────────────
FORWARD_SPEED    = 0.15      # m/s — base waypoint navigation speed
TURN_SPEED       = 1.5       # rad/s — increased for very sharp goal adjustments
AVOID_TURN_SPEED = 1.2       # rad/s — increased for rapid obstacle clearing

# ── Tolerances ─────────────────────────────────────────────────────────────────
WAYPOINT_TOLERANCE  = 0.75   # m — close enough to waypoint to advance
RETURN_TOLERANCE    = 0.15   # m — close enough to origin to stop

# ── Obstacle avoidance ─────────────────────────────────────────────────────────
AVOID_DISTANCE      = 0.3    # m — obstacle trigger distance
FRONT_ARC_DEG       = 80     # degrees — front detection arc width
LIDAR_OFFSET_DEG    = -90    # degrees — LiDAR mounting offset
LIDAR_OFFSET_RAD    = math.radians(LIDAR_OFFSET_DEG)

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
        self.nearest_left      = float('inf')
        self.nearest_right     = float('inf')
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

        # ── Waypoints ────────────────────────────────────────────────────────
        self.waypoints      = self._load_waypoints()
        self.waypoint_index = 0

        # ── State machine ─────────────────────────────────────────────────────
        resume_mode = None
        for arg in sys.argv[1:]:
            if arg.startswith('--resume='):
                resume_mode = arg.split('=', 1)[1].upper()

        if resume_mode == 'RETURN':
            self.state = 'RETURNING'
        elif resume_mode == 'SEARCH':
            self.state = 'WALL_FOLLOWING'
        else:
            self.state = 'WALL_FOLLOWING'

        self.start_time = time.time()
        self.cube_x, self.cube_y, self.return_x, self.return_y = None, None, None, None

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Autonomous search started (Harsh Turning Active)')

    def _load_waypoints(self):
        if PGM_MAP_PATH is None or not os.path.exists(PGM_MAP_PATH):
            return list(MANUAL_WAYPOINTS)
        cylinders = extract_cylinder_waypoints(PGM_MAP_PATH, PGM_MAP_YAML)
        if not cylinders:
            return list(MANUAL_WAYPOINTS)
        return list(cylinders)

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
        """Direct-to-goal with aggressive, high-omega contour following."""
        target_angle = math.atan2(target_y - self.current_y, target_x - self.current_x)
        heading_err = self._angle_diff(target_angle, self.current_yaw)

        # 1. Obstacle immediately ahead -> Stop and pivot immediately
        if self.nearest_front <= AVOID_DISTANCE:
            if self.avoid_dir == 0:
                self.avoid_dir = 1.0 if self.nearest_left >= self.nearest_right else -1.0
                self.get_logger().info(f'[AVOID] Obstacle ahead. Pivoting {"left" if self.avoid_dir > 0 else "right"}')
            
            # Pure spin in place (radius = 0) to clear the obstacle
            return 0.0, self.avoid_dir * AVOID_TURN_SPEED

        # 2. Path ahead is clear
        else:
            if self.avoid_dir != 0:
                # We are tracing an obstacle. Can we resume heading to goal?
                if abs(heading_err) < 0.2:
                    self.get_logger().info('[AVOID] Target alignment clear. Resuming direct heading.')
                    self.avoid_dir = 0
                    return FORWARD_SPEED, 0.0
                else:
                    # TIGHT CONTOUR ARC: Scale down linear velocity to 40% 
                    # and increase angular turn back to 85% of max to severely limit the radius.
                    return FORWARD_SPEED * 0.4, -self.avoid_dir * (AVOID_TURN_SPEED * 0.85)
            
            # 3. Direct to goal mode
            else:
                # Aggressive proportional tracking (gain increased to 3.0)
                angular = float(np.clip(heading_err * 3.0, -TURN_SPEED, TURN_SPEED))
                
                # Scaled linear speed: if making sharp angle corrections, slow down to keep radius harsh
                speed_factor = max(0.2, 1.0 - (abs(heading_err) / (math.pi / 2)))
                return FORWARD_SPEED * speed_factor, angular

    def _start_spin(self):
        self.spin_start_yaw, self.spin_last_yaw = self.current_yaw, self.current_yaw
        self.spin_accumulated, self.cube_detected = 0.0, False
        self.spin_dir = 1.0 if self.nearest_left >= self.nearest_right else -1.0

    def _update_spin(self):
        delta = abs(self._angle_diff(self.current_yaw, self.spin_last_yaw))
        self.spin_accumulated += delta
        self.spin_last_yaw = self.current_yaw
        return self.spin_accumulated >= SPIN_FULL_CIRCLE

    def control_loop(self):
        if not self.odom_received: return

        if self.state in ('WALL_FOLLOWING', 'CUBE_FINDING'):
            if time.time() - self.start_time >= TIME_LIMIT_S:
                self.stop()
                self.state = 'RETURNING'
                return

        # ── WALL_FOLLOWING ───────────────────────────────────────────────────
        if self.state == 'WALL_FOLLOWING':
            if self.waypoint_index >= len(self.waypoints):
                self.stop()
                self.state = 'RETURNING'
                return

            target_x, target_y = self.waypoints[self.waypoint_index]
            dist = self._distance_to(target_x, target_y)

            if dist < WAYPOINT_TOLERANCE:
                self.stop()
                self.avoid_dir = 0 
                self.get_logger().info(f'Waypoint {self.waypoint_index} reached — starting cube scan')
                self._start_spin()
                self.state = 'CUBE_FINDING'
                return

            linear, angular = self._simple_steer(target_x, target_y)
            self._publish_twist(linear, angular)

        # ── CUBE_FINDING ─────────────────────────────────────────────────────
        elif self.state == 'CUBE_FINDING':
            if self.cube_detected:
                self.stop()
                self.avoid_dir = 0
                self.state = 'REPORTING'
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
            self.state = 'RETURNING'

        # ── RETURNING ────────────────────────────────────────────────────────
        elif self.state == 'RETURNING':
            dist = self._distance_to(0.0, 0.0)
            if dist < RETURN_TOLERANCE:
                self.stop()
                self.return_x, self.return_y = self.current_x, self.current_y
                self.state = 'DONE'
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