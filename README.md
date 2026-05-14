# 5/13/26 - latest
## System Requirements & Dependencies

This project is built and tested on **Ubuntu 24.04** running **ROS 2 Jazzy**. 

### 1. Core ROS 2 Packages
The simulation, kinematics, and data collection pipelines rely on the following ROS 2 packages. Install them via `apt`:

```bash
sudo apt update
sudo apt install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-moveit \
    ros-jazzy-moveit-kinematics \
    ros-jazzy-moveit-ros-planning \
    ros-jazzy-moveit-ros-move-group \
    ros-jazzy-rosbag2-storage-mcap \
    python3-pip
```
## C++ Libraries
Eigen3: Required for advanced matrix operations, cross-coupling, and pseudo-inverse calculations in the custom controllers. (Usually pre-installed with ROS 2 Desktop, but available via sudo apt install libeigen3-dev if missing).

Code Architecture Note: Control Logic
Important for Future Developers: The core mathematical control logic for the trajectory trackers (including the PID and Secant algorithms) is implemented as Header-Only C++ code.

You will not find standalone .cpp implementation files for the controller math. All control theory logic, system initializations, state updates, and computational algorithms are housed directly within the .hpp header files (located within the nlc_cpp_lib / include directories).

When modifying controller behavior, adjusting safety velocity bounds, or adding new tracking algorithms, look exclusively inside the .hpp files. The standard .cpp files in this repository are strictly used for ROS 2 node wrappers, subscriber/publisher routing, and hardware bridging.

### Workspace Setup
Clone the repository

```Bash
mkdir -p ~/ros2_ws/
cd ~/ros2_ws/
git clone <repo_url> .
```
### Build the workspace

```Bash
cd ~/ros2_ws
colcon build --symlink-install
```
### Source the environment

```Bash
source ~/ros2_ws/install/setup.bash
```
(Tip: Add source ~/ros2_ws/install/setup.bash to your ~/.bashrc to do this automatically).

### Running the Experiment
Always use the master bash script to collect data. It automatically synchronizes the simulation clock, boots Gazebo, handles the MoveIt IK streamer, executes the 16-second trajectories, records the ROS bags, and cleanly terminates all background processes.

```Bash
cd ~/ros2_ws
chmod +x run_experiment.sh
./run_experiment.sh
```


# 3/25/26 - Old Update below

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
34:    # Start arm controller
35:    start_arm_controller_cmd = ExecuteProcess(
36:        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
37:             'arm_controller'],
38:        output='screen')
...
52:    # NEW Load forward position controller (starts inactive to prevent conflicts)
53:    start_forward_position_controller_cmd = ExecuteProcess(
54:        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
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
