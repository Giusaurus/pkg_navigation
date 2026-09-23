"""
drive_route_action_server.py
"""

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry

from georg_nav_msgs.action import DriveRoute

from .mecanum_controller import (
    Pose2D,
    ControllerGains,
    compute_velocity_command,
    yaw_from_quaternion,
)


ODOM_TOPIC = "/odom"
CMD_VEL_TOPIC = "/cmd_vel"
CONTROL_RATE_HZ = 20.0

ODOM_WAIT_TIMEOUT_S = 5.0


class DriveRouteActionServer(Node):

    def __init__(self, node_name: str = "drive_route_action_server", action_name: str = "/nav/drive_route"):
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
                current_pose, target.x, target.y, gains, is_final_waypoint=is_final
            )

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
    action_name = "/nav/drive_route"
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
