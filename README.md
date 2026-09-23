# pkg_navigation
This is the Navigation Module for the Pib Robot "Georg". It also includes a Drive Module for test porpuses.
It contains two modes for the navigation. The first mode allows a navigation to a specified room (only implemented for later usage).
The mainly used mode is the navigation to an object, which is specified within the map editor.

## Navigate_Action_Server

### Action:     
/nav/navigate_room_x  (room planning)

/nav/navigate_object_x  (object planning)

### Navigate_Room Server:
source install/setup.bash

ros2 run navigate_server navigate_action_server \
  --node-name nav_room_server \
  --action-name /nav/navigate_room_x


### Navigate_Object Server:
source install/setup.bash

ros2 run navigate_server navigate_action_server \
  --node-name nav_object_server \
  --action-name /nav/navigate_object_x


### Optional Parameters:

--ros-args \
  -p map_dir:= ~/georg_maps \
  -p default_map_name:=arena.yaml \
  -p cell_size_m:=0.05


## Drive_Route_Action_Server:

### Subscribes:

/odom  (nav_msgs/Odometry)

### Publishes:

/cmd_vel  (geometry_msgs/Twist)

### Action:     

/nav/drive_route

### Drive_Route_Server:
source install/setup.bash

ros2 run drive_route_server drive_route_action_server

### Optional Parameters:

ros2 run drive_route_server drive_route_action_server --ros-args \
  -p max_linear_speed:=0.4 \
  -p max_angular_speed:=1.5 \
  -p k_linear:=1.0 \
  -p k_angular:=2.0 \
  -p waypoint_tolerance_m:=0.1 \
  -p final_position_tolerance_m:=0.03 \
  -p final_heading_tolerance_rad:=0.15
