#!/bin/bash

# ==========================================
# 1. BUILD & SETUP ENVIRONMENT
# ==========================================
echo "Building Workspace..."
cd ~/ros2_ws 
colcon build --symlink-install
source install/setup.bash
mkdir -p experiment_data

# ==========================================
# 2. START BASE SYSTEMS (Simulation & IK)
# ==========================================
echo "Starting Simulation..."
bash ~/ros2_ws/src/mycobot_ros2/mycobot_bringup/scripts/mycobot_280_gazebo.sh &
SIM_PID=$!
sleep 60
# ASSASSINATE RVIZ: Stop the time-travel flashing and save CPU!
#echo "Killing RViz..."
#pkill -9 rviz2
echo "Starting IK Streamer..."
# Pass sim time to the launch file
ros2 launch mycobot_system_tests ik_test.launch.py use_sim_time:=true &
IK_PID=$!
sleep 15 

NUM_RUNS=3 

# ==========================================
# 3. THE EXPERIMENT LOOP
# ==========================================
for i in $(seq 1 $NUM_RUNS); do
    echo "==================================="
    echo "      STARTING TEST RUN $i         "
    echo "==================================="

    # --------------------------------------
    # A. PID TEST
    # --------------------------------------
    rm -rf "experiment_data/pid_run_$i"

    echo "[Run $i] Starting PID Controller..."
    ~/ros2_ws/install/mycobot_system_tests/lib/mycobot_system_tests/pid --ros-args -p use_sim_time:=true &
    CTRL_PID=$!
    sleep 3

    echo "[Run $i] Recording PID Bag..."
	ros2 bag record -o "experiment_data/pid_run_$i" /joint_states /controller/joint_setpoints /forward_position_controller/commands --use-sim-time &
    BAG_PID=$!
    sleep 1
    echo "[Run $i] Executing Trajectory (Waiting for completion)..."
    python3 ~/ros2_ws/src/mycobot_ros2/mycobot_system_tests/scripts/trajectory_generator.py --ros-args -p use_sim_time:=true

    echo "[Run $i] Trajectory finished! Shutting down PID and saving bag..."
    # FIX 1: Use -2 (SIGINT) for the bag to ensure it saves correctly
# KILL SEQUENCE
    kill -2 $BAG_PID   # Tell bag to save
    kill -9 $CTRL_PID  # Kill PID Controller instantly
    sleep 2
    pkill -9 -f "ros2 bag" # Force close bag if it's hanging
    # 1. Gracefully tell the bag daemon to stop via ROS 2 signal
    ros2 daemon stop
    sleep 2 
    
    # 2. Hard kill the controller

    
    # 3. Guarantee no zombie bag processes remain before starting Secant
    pkill -9 -f "ros2 bag"
    sleep 2 
    ros2 bag reindex "experiment_data/pid_run_$i"
	# --- ADD THIS DEAD TIME ---
    echo "======================================================="
    echo " OBSERVE ARM: PID is DEAD. Arm should be completely limp."
    echo "======================================================="
    sleep 5 
    # --------------------------
    # --------------------------------------
    # B. SECANT TEST
    # --------------------------------------
    rm -rf "experiment_data/secant_run_$i"

    echo "[Run $i] Starting Secant Controller..."
    ~/ros2_ws/install/mycobot_system_tests/lib/mycobot_system_tests/secant --ros-args -p use_sim_time:=true & 
    CTRL_PID=$!
    sleep 3

    echo "[Run $i] Recording Secant Bag..."
    ros2 bag record -o "experiment_data/secant_run_$i" /joint_states /controller/joint_setpoints /forward_position_controller/commands --use-sim-time &
    sleep 1

    echo "[Run $i] Executing Trajectory (Waiting for completion)..."
    python3 ~/ros2_ws/src/mycobot_ros2/mycobot_system_tests/scripts/trajectory_generator.py --ros-args -p use_sim_time:=true

    echo "[Run $i] Trajectory finished! Shutting down Secant and saving bag..."
    ros2 daemon stop
    sleep 2
    kill -9 $CTRL_PID
    pkill -9 -f "ros2 bag"
    sleep 2
    ros2 bag reindex "experiment_data/secant_run_$i"
    # --- ADD THIS DEAD TIME ---
    echo "======================================================="
    echo " OBSERVE ARM: Secant is DEAD. Arm should be completely limp."
    echo "======================================================="
    sleep 5

done

# ==========================================
# 4. CLEANUP
# ==========================================
# ==========================================
# 4. CLEANUP
# ==========================================
echo "Experiment Complete! Cleaning up background processes..."

# 1. Kill the IK node
kill -9 $IK_PID 2>/dev/null

# 2. Nuke the ROS 2 Launch tree and Gazebo Physics Engine
echo "Shutting down Gazebo and ROS 2 background nodes..."
pkill -9 -f "ros2 launch"
pkill -9 -f "gz sim"
pkill -9 -f "ruby" 
pkill -9 -f "robot_state_publisher"
pkill -9 -f "parameter_bridge"
pkill -9 -f "mycobot_280_gazebo.sh"
#echo "Killing RViz..."
pkill -9 rviz2
echo "All data saved to ~/ros2_ws/experiment_data/"
echo "Simulation environment completely sanitized."

