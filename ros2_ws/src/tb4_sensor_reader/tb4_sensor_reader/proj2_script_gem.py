#!/usr/bin/env python3
"""
Autonomous Cube Finder — COMPSYS732
Phase 2 autonomous search node.

States: SEARCHING → REPORTING → RETURNING → DONE

To run:
    ~/ros2_venv/bin/python3 -m tb4_sensor_reader.autonomous_search
"""

import rclpy, cv2, math, os, time
import heapq
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
WAYPOINT_TOLERANCE  = 0.15   # m — close enough to waypoint to advance
HEADING_TOLERANCE   = 0.15   # rad — wide dead-band to prevent oscillation
RETURN_TOLERANCE    = 0.15   # m — close enough to origin to stop

# ── Obstacle avoidance ─────────────────────────────────────────────────────────
AVOID_DISTANCE      = 0.30   # m — obstacle trigger distance
AVOID_CLEAR_DIST    = 0.30   # m — must be clear before resuming navigation
FRONT_ARC_DEG       = 1      # degrees — front detection arc width
LIDAR_OFFSET_DEG    = -90    # degrees — LiDAR mounting offset
LIDAR_OFFSET_RAD    = math.radians(LIDAR_OFFSET_DEG)

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

# ── Path planning ──────────────────────────────────────────────────────────────
ROBOT_RADIUS_M  = 0.20       # m — inflation radius around obstacles
LOS_TOLERANCE   = 5          # px — min pixel distance between simplified waypoints

# ── Map paths ─────────────────────────────────────────────────────────────────
PGM_MAP_PATH = os.path.expanduser('~/Desktop/test_map.pgm')
PGM_MAP_YAML = os.path.expanduser('~/Desktop/test_map.yaml')

# ── Manual waypoint fallback ───────────────────────────────────────────────────
# SWAPPED X and Y
MANUAL_WAYPOINTS = [
    (-2.3, -2.3),
    (0.5, 1.5),
    (1.0, 1.5),
]


# ── A* path planner ────────────────────────────────────────────────────────────

def astar(grid, start, goal):
    """
    A* search on a 2D occupancy grid.
    grid  : 2D numpy array — 0 = free, 255 = occupied
    start : (col, row) pixel tuple
    goal  : (col, row) pixel tuple
    Returns list of (col, row) pixels start→goal, or None if no path found.
    """
    rows, cols = grid.shape
    sc, sr = start
    gc, gr = goal

    if grid[sr, sc] != 0 or grid[gr, gc] != 0:
        return None

    def h(c, r):
        return math.sqrt((c - gc) ** 2 + (r - gr) ** 2)

    # heap entries: (f, g, col, row, path)
    open_heap = []
    heapq.heappush(open_heap, (h(sc, sr), 0.0, sc, sr, []))
    visited = set()

    # 8-connected neighbours
    neighbours = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]

    while open_heap:
        f, g, c, r, path = heapq.heappop(open_heap)
        if (c, r) in visited:
            continue
        visited.add((c, r))
        path = path + [(c, r)]

        if c == gc and r == gr:
            return path

        for dc, dr in neighbours:
            nc, nr = c + dc, r + dr
            if 0 <= nc < cols and 0 <= nr < rows:
                if (nc, nr) not in visited and grid[nr, nc] == 0:
                    ng = g + math.sqrt(dc ** 2 + dr ** 2)
                    heapq.heappush(open_heap, (ng + h(nc, nr), ng, nc, nr, path))

    return None


def simplify_path(pixel_path, grid, tolerance=5):
    """
    Reduce waypoints using line-of-sight simplification (Bresenham).
    Skips intermediate points where direct LOS is unobstructed.
    tolerance: minimum pixel distance between kept waypoints.
    """
    if len(pixel_path) <= 2:
        return pixel_path

    def los_clear(p1, p2):
        c1, r1 = p1
        c2, r2 = p2
        dc = abs(c2 - c1); dr = abs(r2 - r1)
        sc = 1 if c1 < c2 else -1
        sr = 1 if r1 < r2 else -1
        err = dc - dr
        c, r = c1, r1
        while True:
            if grid[r, c] != 0:
                return False
            if c == c2 and r == r2:
                return True
            e2 = 2 * err
            if e2 > -dr: err -= dr; c += sc
            if e2 <  dc: err += dc; r += sr

    simplified = [pixel_path[0]]
    i = 0
    while i < len(pixel_path) - 1:
        j = len(pixel_path) - 1
        while j > i + 1:
            p1, p2 = pixel_path[i], pixel_path[j]
            dist = math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
            if dist >= tolerance and los_clear(p1, p2):
                break
            j -= 1
        simplified.append(pixel_path[j])
        i = j
    return simplified


def plan_path_on_map(pgm_path, yaml_path, start_world, goal_world):
    try:
        import yaml
        with open(yaml_path, 'r') as f:
            meta = yaml.safe_load(f)
        resolution = meta['resolution']
        origin     = meta['origin']

        img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            return None
        h, w = img.shape

        _, occupied = cv2.threshold(img, 50, 255, cv2.THRESH_BINARY_INV)

        radius_px = max(1, int(math.ceil(ROBOT_RADIUS_M / resolution)))
        kernel    = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * radius_px + 1, 2 * radius_px + 1))
        inflated  = cv2.dilate(occupied, kernel)

        # ── SWAPPED W2P AND P2W PER REQUEST ────────────────────────────────
        def w2p(wx, wy):
            # wx is actually the y-axis, wy is the x-axis
            px = int((wy - origin[0]) / resolution)
            py = int(h - (wx - origin[1]) / resolution)
            return max(0, min(w - 1, px)), max(0, min(h - 1, py))

        def p2w(px, py):
            # Return (Y, X) mapped coordinates
            return (
                round(origin[1] + (h - py) * resolution, 3), # X receives Y math
                round(origin[0] + px * resolution, 3)        # Y receives X math
            )
        # ───────────────────────────────────────────────────────────────────

        start_px = w2p(*start_world)
        goal_px  = w2p(*goal_world)

        # Force clear a large starting zone (radius_px * 2) so A* can escape
        cv2.circle(inflated, start_px, radius_px * 2, 0, -1)
        cv2.circle(inflated, goal_px, radius_px + 2, 0, -1)

        print(f'[ASTAR] start_px={start_px} goal_px={goal_px}  '
              f'start_free={inflated[start_px[1], start_px[0]]==0}  '
              f'goal_free={inflated[goal_px[1], goal_px[0]]==0}')

        # Nudge goal if it lands inside an inflated obstacle
        if inflated[goal_px[1], goal_px[0]] != 0:
            print(f'[ASTAR] Goal pixel is occupied — searching for nearest free cell')
            found = False
            for radius in range(1, radius_px * 3):
                for dc in range(-radius, radius + 1):
                    for dr in range(-radius, radius + 1):
                        if abs(dc) != radius and abs(dr) != radius:
                            continue
                        nc = max(0, min(w - 1, goal_px[0] + dc))
                        nr = max(0, min(h - 1, goal_px[1] + dr))
                        if inflated[nr, nc] == 0:
                            print(f'[ASTAR] Nudged goal from {goal_px} → ({nc},{nr})')
                            goal_px = (nc, nr)
                            found = True
                            break
                    if found:
                        break
                if found:
                    break
            if not found:
                print('[ASTAR] Could not find free cell near goal — aborting')
                return None

        # Check start is free
        if inflated[start_px[1], start_px[0]] != 0:
            print(f'[ASTAR] Start pixel is occupied — cannot plan')
            return None

        pixel_path = astar(inflated, start_px, goal_px)
        if pixel_path is None:
            print(f'[ASTAR] No path found between {start_px} and {goal_px}')
            return None

        print(f'[ASTAR] Raw path: {len(pixel_path)} px  → simplifying')
        simplified = simplify_path(pixel_path, inflated, tolerance=LOS_TOLERANCE)
        print(f'[ASTAR] Simplified path: {len(simplified)} waypoints')

        return [p2w(px, py) for px, py in simplified[1:]]

    except Exception as e:
        print(f'[ASTAR] Unexpected exception: {e}')
        return None

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

        # Wall is the largest component — everything else is a candidate
        wall_label = 1 + int(np.argmax(
            [stats[i, cv2.CC_STAT_AREA] for i in range(1, num_labels)]))
        wall_area  = stats[wall_label, cv2.CC_STAT_AREA]
        print(f'[EXTRACT] Wall component: label={wall_label} area={wall_area}')

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

            # ── SWAPPED WORLD X AND Y PER REQUEST ────────────────────────────
            world_x = origin[1] + (h - cy) * resolution # X assigned the Y math
            world_y = origin[0] + cx * resolution       # Y assigned the X math
            # ─────────────────────────────────────────────────────────────────
            
            waypoints.append((round(world_x, 3), round(world_y, 3)))
            print(f' → accepted  swapped_world=({world_x:.3f},{world_y:.3f})')

        # Debug image
        try:
            SCALE = 8
            debug_img = cv2.cvtColor(
                cv2.resize(binary, (w * SCALE, h * SCALE),
                           interpolation=cv2.INTER_NEAREST),
                cv2.COLOR_GRAY2BGR)
            for (wx, wy) in waypoints:
                # Un-swap to draw correctly on the original map grid
                px_d = int((wy - origin[0]) / resolution) * SCALE
                py_d = int(h - (wx - origin[1]) / resolution) * SCALE
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
        self.current_x      = 0.0
        self.current_y      = 0.0
        self.current_yaw    = 0.0
        self.nearest_front  = float('inf')
        self.nearest_left   = float('inf')
        self.nearest_right  = float('inf')
        self.latest_scan    = None
        self.latest_image   = None
        self.cube_detected  = False

        # ── Waypoints ────────────────────────────────────────────────────────
        self.waypoints      = self._load_waypoints()
        self.waypoint_index = 0

        # ── Obstacle avoidance memory ─────────────────────────────────────────
        self.avoiding        = False
        self.avoid_direction = 0         # +1 = left, -1 = right

        # ── State machine ─────────────────────────────────────────────────────
        self.state      = 'SEARCHING'
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

    # ── Waypoint loading ──────────────────────────────────────────────────────

    def _load_waypoints(self):
        """
        Extract cylinder positions from PGM map, plan an A* path from (0,0)
        through each cylinder in order, and return the full concatenated
        world-frame waypoint list. Falls back to MANUAL_WAYPOINTS on failure.
        """
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
            f'Found {len(cylinders)} cylinders: {cylinders}')

        full_path     = []
        current_start = (0.0, 0.0)

        for i, cyl in enumerate(cylinders):
            self.get_logger().info(f'Planning A* path to cylinder {i}: {cyl}')
            segment = plan_path_on_map(
                PGM_MAP_PATH, PGM_MAP_YAML, current_start, cyl)

            if segment:
                self.get_logger().info(
                    f'  Path found — {len(segment)} waypoints')
                full_path.extend(segment)
            else:
                self.get_logger().warn(
                    f'  A* failed for cylinder {i} — inserting direct waypoint')
                full_path.append(cyl)

            current_start = cyl

        if not full_path:
            self.get_logger().warn(
                'Path planning produced no waypoints — using manual')
            return list(MANUAL_WAYPOINTS)

        # ── VISUAL PATH DEBUGGING ─────────────────────────────────────────────
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
                    cv2.resize(img, (w * SCALE, h * SCALE), interpolation=cv2.INTER_NEAREST),
                    cv2.COLOR_GRAY2BGR)

                def w2p_scaled(wx, wy):
                    # Un-swap to draw correctly on the original map image layout
                    px = int((wy - origin[0]) / resolution)
                    py = int(h - (wx - origin[1]) / resolution)
                    return (max(0, min(w - 1, px)) * SCALE, max(0, min(h - 1, py)) * SCALE)

                for cyl in cylinders:
                    cv2.circle(debug_img, w2p_scaled(*cyl), 10, (0, 0, 255), -1)

                origin_px = w2p_scaled(0.0, 0.0)
                
                # Axes labels adapted to the swapped logic
                x_axis_px = w2p_scaled(0.4, 0.0)
                cv2.arrowedLine(debug_img, origin_px, x_axis_px, (0, 0, 255), 3, tipLength=0.2)
                cv2.putText(debug_img, 'X(Swapped)', (x_axis_px[0] + 5, x_axis_px[1]), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                
                y_axis_px = w2p_scaled(0.0, 0.4)
                cv2.arrowedLine(debug_img, origin_px, y_axis_px, (0, 255, 0), 3, tipLength=0.2)
                cv2.putText(debug_img, 'Y(Swapped)', (y_axis_px[0] - 5, y_axis_px[1] - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                cv2.circle(debug_img, origin_px, 6, (255, 0, 0), -1)

                path_sequence = [(0.0, 0.0)] + full_path
                for i in range(len(path_sequence) - 1):
                    pt1 = w2p_scaled(*path_sequence[i])
                    pt2 = w2p_scaled(*path_sequence[i+1])
                    cv2.arrowedLine(debug_img, pt1, pt2, (255, 255, 0), 2, tipLength=0.03)
                    cv2.circle(debug_img, pt2, 4, (255, 0, 0), -1)

                debug_path = os.path.expanduser('~/Desktop/path_debug.jpg')
                ok = cv2.imwrite(debug_path, debug_img)
                if ok:
                    self.get_logger().info(f'[DEBUG] Path map saved → {debug_path}')
        except Exception as e:
            self.get_logger().warn(f'[DEBUG] Exception generating path image: {e}')
        # ──────────────────────────────────────────────────────────────────────

        self.get_logger().info(
            f'Total planned path: {len(full_path)} waypoints')
        return full_path
    

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

    # ── State machine ─────────────────────────────────────────────────────────

    def control_loop(self):

        # ── Time limit ───────────────────────────────────────────────────────
        if self.state == 'SEARCHING':
            if time.time() - self.start_time >= TIME_LIMIT_S:
                self.get_logger().warn(
                    f'Time limit reached — transitioning to RETURNING')
                self.stop()
                self.state = 'RETURNING'
                return

        # ── SEARCHING ────────────────────────────────────────────────────────
        if self.state == 'SEARCHING':

            if self.cube_detected:
                self.stop()
                self.get_logger().info('Red cube detected — transitioning to REPORTING')
                self.state = 'REPORTING'
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
                self.stop()
                self.get_logger().info(
                    f'Waypoint {self.waypoint_index} reached '
                    f'({target_x:.2f}, {target_y:.2f})')
                self.waypoint_index += 1
                return

            # ── Obstacle avoidance ───────────────────────────────────────────
            if self.nearest_front <= AVOID_DISTANCE and not self.avoiding:
                self.avoiding = True
                if self.nearest_left >= self.nearest_right:
                    self.avoid_direction = +1
                    self.get_logger().warn('Obstacle — COMMIT LEFT')
                else:
                    self.avoid_direction = -1
                    self.get_logger().warn('Obstacle — COMMIT RIGHT')

            if self.avoiding:
                if self.nearest_front > AVOID_CLEAR_DIST:
                    self.avoiding = False
                    self.get_logger().info('Obstacle cleared')
                else:
                    self._publish_twist(0.0, self.avoid_direction * AVOID_TURN_SPEED)
                    return

            # ── Steer toward waypoint ────────────────────────────────────────
            target_angle = math.atan2(
                target_y - self.current_y,
                target_x - self.current_x)
            heading_err = self._angle_diff(target_angle, self.current_yaw)

            if abs(heading_err) > HEADING_TOLERANCE:
                scale = min(1.0, abs(heading_err) / math.pi)
                turn  = math.copysign(TURN_SPEED * scale, heading_err)
                self._publish_twist(0.0, turn)
            else:
                self._publish_twist(FORWARD_SPEED, 0.0)

            self.get_logger().info(
                f'→ WP{self.waypoint_index} ({target_x:.2f},{target_y:.2f}) '
                f'dist={dist:.2f} m  hdg_err={math.degrees(heading_err):.1f}°  '
                f'pos=({self.current_x:.2f},{self.current_y:.2f})')

        # ── REPORTING ────────────────────────────────────────────────────────
        elif self.state == 'REPORTING':
            self.stop()
            
            # COMPLIANCE FIX: Log exact robot odometry as the cube location
            self.cube_x = self.current_x
            self.cube_y = self.current_y
            
            self.get_logger().info(
                f'Cube position logged at odometry: x={self.cube_x:.3f} m  y={self.cube_y:.3f} m  '
                f'(yaw={math.degrees(self.current_yaw):.1f}°)')
                
            self._save_snapshot()
            self.state = 'RETURNING'

        # ── RETURNING ────────────────────────────────────────────────────────
        elif self.state == 'RETURNING':
            dx   = 0.0 - self.current_x
            dy   = 0.0 - self.current_y
            dist = math.sqrt(dx ** 2 + dy ** 2)

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
                scale = min(1.0, abs(heading_err) / math.pi)
                turn  = math.copysign(TURN_SPEED * scale, heading_err)
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