"""
drive_route_action_server.py
"""

import time
import math
import rclpy
import yaml

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from georg_nav_msgs.action import DriveRoute
from sensor_msgs.msg import LaserScan

from .mecanum_controller import (
    Pose2D,
    ControllerGains,
    compute_velocity_command,
    yaw_from_quaternion,
)


ODOM_TOPIC = "odom"
CMD_VEL_TOPIC = "cmd_vel"
CONTROL_RATE_HZ = 20.0

ODOM_WAIT_TIMEOUT_S = 5.0

SCAN_TOPIC = "scan"

OBSTACLE_CONE_HALF_ANGLE_DEG = 30.0
OBSTACLE_STOP_DISTANCE_M = 0.8
OBSTACLE_REPLAN_STOP_DISTANCE_M = 0.5
OBSTACLE_REPLAN_WINDOW_S = 15.0
OBSTACLE_CORRIDOR_HALF_WIDTH_M = 0.20
OBSTACLE_REPLAN_GRACE_M = 0.25
OBSTACLE_MIN_COMMANDED_SPEED = 0.05
OBSTACLE_REPORT_RANGE_M = 2.0
OBSTACLE_REPORT_HALF_ANGLE_DEG = 90.0
OBSTACLE_POINT_GRID_M = 0.1
OBSTACLE_MAX_POINTS = 150
KNOWN_OBSTACLE_MARGIN_M = 0.35
OBSTACLE_DETECTION_ENABLED = True

MAP_FILE = "/home/giu/georg_maps/Nav_Test.yaml"

def _load_known_polygons(path):
    try:
        with open(path) as f:
            data = yaml.safe_load(f)
    except Exception:
        return []
    polys = []
    for p in data.get('polygons', []):
        pts = [tuple(pt) for pt in p.get('points', [])]
        if len(pts) >= 3:
            polys.append((p.get('type', 'obstacle'), pts))
    return polys


def _dist_to_segment(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / l2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _point_in_polygon(px, py, pts):
    inside = False
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        if (y1 > py) != (y2 > py) and px < (x2 - x1) * (py - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def _near_known_polygon(px, py, polys, margin):
    for kind, pts in polys:
        n = len(pts)
        if min(_dist_to_segment(px, py, *pts[i], *pts[(i + 1) % n]) for i in range(n)) <= margin:
            return True
        if kind != 'boundary' and _point_in_polygon(px, py, pts):
            return True
    return False

class DriveRouteActionServer(Node):

    def __init__(self, node_name: str = "drive_route_action_server", action_name: str = "nav/drive_route"):
        super().__init__(node_name)

        self.declare_parameter("k_linear", ControllerGains.k_linear)
        self.declare_parameter("k_angular", ControllerGains.k_angular)
        self.declare_parameter("max_linear_speed", ControllerGains.max_linear_speed)
        self.declare_parameter("max_angular_speed", ControllerGains.max_angular_speed)
        self.declare_parameter("waypoint_tolerance_m", ControllerGains.waypoint_tolerance_m)
        self.declare_parameter("final_position_tolerance_m", ControllerGains.final_position_tolerance_m)
        self.declare_parameter("final_heading_tolerance_rad", ControllerGains.final_heading_tolerance_rad)

        self._gains = ControllerGains(
            k_linear=self.get_parameter("k_linear").value,
            k_angular=self.get_parameter("k_angular").value,
            max_linear_speed=self.get_parameter("max_linear_speed").value,
            max_angular_speed=self.get_parameter("max_angular_speed").value,
            waypoint_tolerance_m=self.get_parameter("waypoint_tolerance_m").value,
            final_position_tolerance_m=self.get_parameter("final_position_tolerance_m").value,
            final_heading_tolerance_rad=self.get_parameter("final_heading_tolerance_rad").value,
        )

        self._latest_pose: Pose2D | None = None

        odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._odom_sub = self.create_subscription(
            Odometry, ODOM_TOPIC, self._odom_callback, odom_qos
        )

        self._latest_scan = None
        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._scan_sub = self.create_subscription(
            LaserScan, SCAN_TOPIC, self._scan_callback, scan_qos
        )


        self._cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        self._callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            DriveRoute,
            action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            f"DriveRouteActionServer ready as node '{node_name}' on action "
            f"'{action_name}' (odom={ODOM_TOPIC}, cmd_vel={CMD_VEL_TOPIC})"
        )

    
    # Pose tracking
    

    def _odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(ori.x, ori.y, ori.z, ori.w)
        self._latest_pose = Pose2D(x=pos.x, y=pos.y, yaw=yaw)

    def _wait_for_odom(self, timeout_s: float) -> bool:

        start = time.monotonic()
        while self._latest_pose is None:
            if time.monotonic() - start > timeout_s:
                return False
            time.sleep(0.05)
        return True



    def _scan_callback(self, msg: LaserScan):
        self._latest_scan = msg

    def _is_known(self, r, angle):
        
        pose = getattr(self, '_latest_pose', None)
        if pose is None:
            return False
        h = pose.yaw + angle
        return _near_known_polygon(pose.x + r * math.cos(h), pose.y + r * math.sin(h),
                                   getattr(self, '_known_polys', []), KNOWN_OBSTACLE_MARGIN_M)

    

    def _min_range_in_cone(self, direction_rad: float, max_dist: float = OBSTACLE_STOP_DISTANCE_M):
        # Kleinste Distanz im Fahrkorridor in Fahrtrichtung (Roboterframe).
        # Rueckgabe: (distanz, winkel) oder None.
        scan = self._latest_scan
        if scan is None:
            return None
        cd, sd = math.cos(direction_rad), math.sin(direction_rad)
        best = None
        for i, r in enumerate(scan.ranges):
            if not (scan.range_min <= r <= scan.range_max):
                continue
            angle = scan.angle_min + i * scan.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            along = x * cd + y * sd
            lateral = -x * sd + y * cd
            if along <= 0.3 or along > max_dist \
                    or abs(lateral) > OBSTACLE_CORRIDOR_HALF_WIDTH_M:
                continue
            if (best is None or along < best[0]) and not self._is_known(r, angle):
                best = (along, angle)
        return best


    def _obstacle_points(self, pose, direction_rad: float):
        # Alle Scan-Treffer im Kegel um die Fahrtrichtung als Weltpunkte,
        # auf ein grobes Raster verduennt, naechste zuerst.
        scan = self._latest_scan
        if scan is None:
            return []

        half = math.radians(OBSTACLE_REPORT_HALF_ANGLE_DEG)
        max_r = min(scan.range_max, OBSTACLE_REPORT_RANGE_M)
        cells = {}
        for i, r in enumerate(scan.ranges):
            if not (scan.range_min <= r <= max_r):
                continue
            angle = scan.angle_min + i * scan.angle_increment
            diff = math.atan2(math.sin(angle - direction_rad),
                              math.cos(angle - direction_rad))
            
            if abs(diff) > half:
                continue

            if self._is_known(r, angle):
                continue

            heading = pose.yaw + angle
            x = pose.x + r * math.cos(heading)
            y = pose.y + r * math.sin(heading)
            key = (round(x / OBSTACLE_POINT_GRID_M), round(y / OBSTACLE_POINT_GRID_M))
            if key not in cells or r < cells[key][0]:
                cells[key] = (r, x, y)

        nearest = sorted(cells.values())[:OBSTACLE_MAX_POINTS]
        return [Point(x=x, y=y, z=0.0) for _, x, y in nearest]
    
    # Goal / cancel acceptance


    def goal_callback(self, goal_request):
        self.get_logger().info(
            f"DriveRoute goal received: {len(goal_request.waypoints)} waypoints"
        )
        if len(goal_request.waypoints) == 0:
            self.get_logger().warn("DriveRoute goal has zero waypoints, rejecting")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("DriveRoute cancel requested")
        return CancelResponse.ACCEPT

    
    # Execute
    

    def execute_callback(self, goal_handle):
        request = goal_handle.request
        result = DriveRoute.Result()

        waypoints: list[Point] = list(request.waypoints)
        self._known_polys = _load_known_polygons(request.map_name or MAP_FILE)


        if not self._wait_for_odom(ODOM_WAIT_TIMEOUT_S):
            self.get_logger().error(
                f"No {ODOM_TOPIC} message received within "
                f"{ODOM_WAIT_TIMEOUT_S}s -- cannot drive without pose"
            )
            result.result_code = DriveRoute.Result.RESULT_ABORTED
            goal_handle.abort()
            return result

        max_speed = request.max_speed if request.max_speed > 0.0 else None
        gains = self._gains
        if max_speed is not None:
            gains = ControllerGains(
                k_linear=self._gains.k_linear,
                k_angular=self._gains.k_angular,
                max_linear_speed=max_speed,
                max_angular_speed=self._gains.max_angular_speed,
                waypoint_tolerance_m=self._gains.waypoint_tolerance_m,
                final_position_tolerance_m=self._gains.final_position_tolerance_m,
                final_heading_tolerance_rad=self._gains.final_heading_tolerance_rad,
            )

        hold_yaw = self._latest_pose.yaw
        start_pose = self._latest_pose


        period_s = 1.0 / CONTROL_RATE_HZ
        wp_idx = 0

        while wp_idx < len(waypoints):
            loop_start = time.monotonic()

            if goal_handle.is_cancel_requested:
                self._publish_stop()
                self.get_logger().info("DriveRoute canceled mid-route")
                result.result_code = DriveRoute.Result.RESULT_ABORTED
                goal_handle.canceled()
                return result

            current_pose = self._latest_pose
            if current_pose is None:
                # stops robot when odom drops out mid route
                self._publish_stop()
                self.get_logger().error("Lost odometry mid-route, aborting")
                result.result_code = DriveRoute.Result.RESULT_ABORTED
                goal_handle.abort()
                return result

            target = waypoints[wp_idx]
            is_final = (wp_idx == len(waypoints) - 1)


            vel, reached = compute_velocity_command(
                current_pose, target.x, target.y, gains, is_final_waypoint=is_final,
                target_yaw=hold_yaw,
            )

            speed = math.hypot(vel.vx, vel.vy)
            near_goal = is_final and (
                math.hypot(target.x - current_pose.x, target.y - current_pose.y)
                < 0.4
            )

            in_replan = (time.monotonic() - getattr(self, '_last_obstacle_time', -1e9)
                        < OBSTACLE_REPLAN_WINDOW_S)
            in_grace = in_replan and math.hypot(current_pose.x - start_pose.x,
                                                current_pose.y - start_pose.y) < OBSTACLE_REPLAN_GRACE_M

            if OBSTACLE_DETECTION_ENABLED and speed > OBSTACLE_MIN_COMMANDED_SPEED \
                    and not near_goal and not in_grace:
                
                direction = math.atan2(vel.vy, vel.vx)
                stop_dist = (OBSTACLE_REPLAN_STOP_DISTANCE_M
                             if time.monotonic() - getattr(self, '_last_obstacle_time', -1e9) < OBSTACLE_REPLAN_WINDOW_S
                             else OBSTACLE_STOP_DISTANCE_M)
                
                hit = self._min_range_in_cone(direction, stop_dist)
                if hit is not None and hit[0] < stop_dist:

                    dist, bearing = hit
                    heading = current_pose.yaw + bearing
                    self._publish_stop()
                    self._last_obstacle_time = time.monotonic()
                    self.get_logger().warn(
                        f"Obstacle at {dist:.2f} m, stopping | Fahrtrichtung={math.degrees(direction):.0f} Grad, "
                        f"Treffer={math.degrees(bearing):.0f} Grad, yaw={math.degrees(current_pose.yaw):.0f} Grad, "
                        f"vx={vel.vx:.2f} vy={vel.vy:.2f}"
                    )
                    result.result_code = DriveRoute.Result.RESULT_OBSTACLE_DETECTED
                    result.blocked_position = Point(
                        x=current_pose.x + dist * math.cos(heading),
                        y=current_pose.y + dist * math.sin(heading),
                        z=0.0,
                    )
                    result.blocked_points = self._obstacle_points(current_pose, direction)
                    goal_handle.abort()
                    return result


            self._publish_velocity(vel)

            feedback = DriveRoute.Feedback()
            feedback.current_waypoint_index = wp_idx
            feedback.current_position = Point(x=current_pose.x, y=current_pose.y, z=0.0)
            goal_handle.publish_feedback(feedback)

            if reached:
                wp_idx += 1

            elapsed = time.monotonic() - loop_start
            sleep_time = period_s - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        self._publish_stop()
        result.result_code = DriveRoute.Result.RESULT_OK
        goal_handle.succeed()
        self.get_logger().info("DriveRoute succeeded: all waypoints reached")
        return result

    
    # cmd_vel publishing
    

    def _publish_velocity(self, vel):
        msg = Twist()
        msg.linear.x = vel.vx
        msg.linear.y = vel.vy
        msg.angular.z = vel.omega
        self._cmd_vel_pub.publish(msg)

    def _publish_stop(self):
        self._cmd_vel_pub.publish(Twist())


def main(args=None):
    import sys

    raw_args = sys.argv[1:] if args is None else args

    node_name = "drive_route_action_server"
    action_name = "nav/drive_route"
    filtered_args = []
    i = 0
    while i < len(raw_args):
        if raw_args[i] == "--node-name" and i + 1 < len(raw_args):
            node_name = raw_args[i + 1]
            i += 2
        elif raw_args[i] == "--action-name" and i + 1 < len(raw_args):
            action_name = raw_args[i + 1]
            i += 2
        else:
            filtered_args.append(raw_args[i])
            i += 1

    rclpy.init(args=filtered_args)

    node = DriveRouteActionServer(node_name=node_name, action_name=action_name)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
