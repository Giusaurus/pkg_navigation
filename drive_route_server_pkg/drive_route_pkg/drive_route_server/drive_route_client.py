"""
drive_route_client.py

"""

import sys
import time
import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point
from georg_nav_msgs.action import DriveRoute


class DriveRouteClient(Node):

    def __init__(self, action_name: str = "/nav/drive_route"):
        super().__init__("drive_route_client")
        self._client = ActionClient(self, DriveRoute, action_name)
        self._action_name = action_name

    def send_route_and_wait(
        self,
        waypoints: list[tuple[float, float]],
        max_speed: float = 0.0,
        server_wait_timeout_s: float = 10.0,
        feedback_callback=None,
    ):

        if not self._client.wait_for_server(timeout_sec=server_wait_timeout_s):
            self.get_logger().error(
                f"DriveRoute server on '{self._action_name}' not available "
                f"after {server_wait_timeout_s}s"
            )
            return None

        goal_msg = DriveRoute.Goal()
        goal_msg.waypoints = [Point(x=x, y=y, z=0.0) for x, y in waypoints]
        goal_msg.max_speed = max_speed

        send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=(
                (lambda _handle, feedback: feedback_callback(feedback))
                if feedback_callback else None
            ),
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("DriveRoute goal was rejected")
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        return result_future.result().result


def _cli_main():

    if len(sys.argv) < 2:
        print("Usage: drive_route_client <x1,y1> [x2,y2 ...] [--action-name NAME]")
        sys.exit(1)

    action_name = "/nav/drive_route"
    coord_args = []
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        if args[i] == "--action-name" and i + 1 < len(args):
            action_name = args[i + 1]
            i += 2
        else:
            coord_args.append(args[i])
            i += 1

    waypoints = []
    for arg in coord_args:
        x_str, y_str = arg.split(",")
        waypoints.append((float(x_str), float(y_str)))

    rclpy.init()
    client = DriveRouteClient(action_name=action_name)

    def on_feedback(feedback):
        print(
            f"  feedback: waypoint {feedback.current_waypoint_index}, "
            f"pos=({feedback.current_position.x:.2f}, "
            f"{feedback.current_position.y:.2f})"
        )

    print(f"Sending {len(waypoints)} waypoints to '{action_name}'...")
    result = client.send_route_and_wait(waypoints, feedback_callback=on_feedback)

    if result is None:
        print("FAILED: goal rejected or server unavailable")
    elif result.result_code == DriveRoute.Result.RESULT_OK:
        print("SUCCESS: route completed")
    else:
        print(f"FAILED: result_code={result.result_code}")

    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    _cli_main()
