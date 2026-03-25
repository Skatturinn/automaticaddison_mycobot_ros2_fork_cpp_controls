

# Alias run & build commands
build and run commands, this assumes you have ros2 setup

## ~/.bashrc
added the following alias
```
alias robot='bash ~/ros2_ws/src/mycobot_ros2/mycobot_bringup/scripts/mycobot_280_gazebo.sh'
alias rosbuild='cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash'
```


# Meaningful Changes to original

## Adding Forward command controller
Added forward command controller to ROS2 controllers for simulation
### files:
[src/mycobot_ros2/mycobot_moveit_config/config/mycobot_280/ros2_controllers.yaml](src/mycobot_ros2/mycobot_moveit_config/config/mycobot_280/ros2_controllers.yaml)
[src/mycobot_ros2/mycobot_moveit_config/config/mycobot_280/ros2_controllers_template.yaml](src/mycobot_ros2/mycobot_moveit_config/config/mycobot_280/ros2_controllers_template.yaml)
## Changes:
#### Added to control manager list:
```
 3:controller_manager:
 4:  ros__parameters:
...
21:    forward_position_controller:
22:      type: forward_command_controller/ForwardCommandController
```
#### Added controler parameters:
```
57:forward_position_controller:
58:  ros__parameters:
59:    joints:
60:      - link1_to_link2
61:      - link2_to_link3
62:      - link3_to_link4
63:      - link4_to_link5
64:      - link5_to_link6
65:      - link6_to_link6_flange
66:    interface_name: position
```

## Loading Forward command controller


### files:
[src/mycobot_ros2/mycobot_moveit_config/launch/load_ros2_controllers.launch.py](src/mycobot_ros2/mycobot_moveit_config/launch/load_ros2_controllers.launch.py)
### changes

```
28:def generate_launch_description():
...
52:    # NEW Load forward position controller (starts inactive to prevent conflicts)
53:    start_forward_position_controller_cmd = ExecuteProcess(
54:        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
55:             'forward_position_controller'],
56:        output='screen')
57:
58:    # Add delay to joint state broadcaster (if necessary)
59:    delayed_start = TimerAction(
60:        period=15.0,  # <- Increaded to allow forward command controller to load
61:        actions=[start_joint_state_broadcaster_cmd]
61:    )
...
77:    # NEW Load the forward position controller after launching the gripper controller
78:    load_forward_position_controller_cmd = RegisterEventHandler(
79:        event_handler=OnProcessExit(
80:            target_action=start_gripper_action_controller_cmd,
81:            on_exit=[start_forward_position_controller_cmd]))
...
91:    ld.add_action(load_forward_position_controller_cmd) # <-- Added to sequence

```

## Testing Forward command controller setup

### Verify Status: 
```
ros2 control list_controllers
```

### Swap Controllers: 
if it's inactive
```
ros2 control switch_controllers --activate forward_position_controller --deactivate arm_controller
```


### Test via Terminal: 
```
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" -1
```
