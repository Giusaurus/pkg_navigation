"""
navigate_action_server.py

"""

import os
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Point
from georg_nav_msgs.action import Navigate

from . import grid_rasterizer
from . import astar


DEFAULT_MAP_DIR = os.environ.get("GEORG_MAP_DIR", os.path.expanduser("~/georg_maps"))
DEFAULT_MAP_NAME = os.environ.get("GEORG_DEFAULT_MAP", "arena.yaml")
DEFAULT_NODE_NAME = "navigate_action_server"
DEFAULT_ACTION_NAME = "navigate"


class NavigateActionServer(Node):
    #Planning-only Navigate.action server.

    #    navigate_action_server --ros-args -p action_name:=/nav/navigate_room_x
    #    navigate_action_server --ros-args -p action_name:=/nav/navigate_object_x
    

    def __init__(self, node_name: str = DEFAULT_NODE_NAME, action_name: str = DEFAULT_ACTION_NAME):
        super().__init__(node_name)

        self.declare_parameter("map_dir", DEFAULT_MAP_DIR)
        self.declare_parameter("default_map_name", DEFAULT_MAP_NAME)
        self.declare_parameter("cell_size_m", grid_rasterizer.DEFAULT_CELL_SIZE_M)

        self._map_dir = self.get_parameter("map_dir").value
        self._default_map_name = self.get_parameter("default_map_name").value
        self._cell_size_m = self.get_parameter("cell_size_m").value

        # Cache loaded grids by map name so repeated goals against the same
        # map do not re-rasterize every time.
        self._grid_cache = {}
        self._grid_cache_lock = threading.Lock()

        self._callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            Navigate,
            action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            f"NavigateActionServer ready as node '{node_name}' on action "
            f"'{action_name}' (map_dir={self._map_dir}, "
            f"default_map={self._default_map_name}, cell_size={self._cell_size_m})"
        )


    # Goal / cancel acceptance


    def goal_callback(self, goal_request):
        self.get_logger().info(
            f"Navigate goal received: start=({goal_request.start.x:.2f}, "
            f"{goal_request.start.y:.2f}) goal=({goal_request.goal.x:.2f}, "
            f"{goal_request.goal.y:.2f})"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Navigate cancel requested")
        return CancelResponse.ACCEPT


    # Map loading


    def _resolve_map_path(self, map_name: str) -> str:
        name = map_name or self._default_map_name
        if os.path.isabs(name):
            return name
        return os.path.join(self._map_dir, name)

    def _get_grid(self, map_name: str):
        path = self._resolve_map_path(map_name)

        with self._grid_cache_lock:
            cached = self._grid_cache.get(path)
            if cached is not None:
                return cached

        if not os.path.isfile(path):
            return None

        grid = grid_rasterizer.load_grid(path, cell_size_m=self._cell_size_m)

        with self._grid_cache_lock:
            self._grid_cache[path] = grid

        return grid


    # Execute


    def execute_callback(self, goal_handle):
        request = goal_handle.request
        result = Navigate.Result()

        grid = self._get_grid(request.map_name)
        if grid is None:
            self.get_logger().error(
                f"Map not loaded/found (map_name='{request.map_name}')"
            )
            result.result_code = Navigate.Result.RESULT_MAP_NOT_LOADED
            goal_handle.abort()
            return result

        start_cell = grid.world_to_cell(request.start.x, request.start.y)
        goal_cell = grid.world_to_cell(request.goal.x, request.goal.y)

        if not grid.is_free(*start_cell):
            self.get_logger().warn(
                f"Start position ({request.start.x:.2f}, {request.start.y:.2f}) "
                "is in an occupied/out-of-bounds cell"
            )
            result.result_code = Navigate.Result.RESULT_START_BLOCKED
            goal_handle.abort()
            return result

        if not grid.is_free(*goal_cell):
            self.get_logger().warn(
                f"Goal position ({request.goal.x:.2f}, {request.goal.y:.2f}) "
                "is in an occupied/out-of-bounds cell"
            )
            result.result_code = Navigate.Result.RESULT_GOAL_BLOCKED
            goal_handle.abort()
            return result

        def abort_check():
            return goal_handle.is_cancel_requested

        def feedback_cb(cells_expanded):
            feedback = Navigate.Feedback()
            feedback.cells_expanded = cells_expanded
            goal_handle.publish_feedback(feedback)

        cell_path = astar.find_path(
            grid,
            start_cell,
            goal_cell,
            abort_check=abort_check,
            feedback_callback=feedback_cb,
        )

        if goal_handle.is_cancel_requested:
            self.get_logger().info("Navigate goal canceled mid-search")
            result.result_code = Navigate.Result.RESULT_ABORTED
            goal_handle.canceled()
            return result

        if cell_path is None:
            self.get_logger().warn("A* found no path between start and goal")
            result.result_code = Navigate.Result.RESULT_NO_PATH_FOUND
            goal_handle.abort()
            return result

        if request.simplify_path:
            cell_path = astar.simplify_path(cell_path, grid=grid)

        waypoints = []
        path_length_m = 0.0
        prev_world = None
        for col, row in cell_path:
            wx, wy = grid.cell_to_world(col, row)
            waypoints.append(Point(x=wx, y=wy, z=0.0))
            if prev_world is not None:
                path_length_m += ((wx - prev_world[0]) ** 2 + (wy - prev_world[1]) ** 2) ** 0.5
            prev_world = (wx, wy)

        result.result_code = Navigate.Result.RESULT_OK
        result.waypoints = waypoints
        result.path_length_m = path_length_m

        goal_handle.succeed()
        self.get_logger().info(
            f"Navigate succeeded: {len(waypoints)} waypoints, "
            f"{path_length_m:.2f} m"
        )
        return result


def main(args=None):
    import sys

    raw_args = sys.argv[1:] if args is None else args

    # Usage:
    # navigate_action_server --node-name nav_room_server --action-name /nav/navigate_room_x
    
    node_name = DEFAULT_NODE_NAME
    action_name = DEFAULT_ACTION_NAME
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

    node = NavigateActionServer(node_name=node_name, action_name=action_name)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()