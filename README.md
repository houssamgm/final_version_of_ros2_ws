# Installation
## Clone the repository
## 1. Clone the repository

 git clone https://github.com/houssamgm/final_version_of_ros2_ws.git
 
 cd ros2_ws
 
## 2. Install dependencies

rosdep update

rosdep install --from-paths src --ignore-src -r -y

## 3. Build the workspace

colcon build

## 4. Source the workspace

source install/setup.bash


# how to run it # diffbot_sim — Working Nav2 + SLAM Run Commands 

## Terminal 1 — Simulation & Environment

### Nav2 (old behavior):

ros2 launch diffbot_sim sim.launch.py use_nav2:=true
ros2 run diffbot_sim cmd_vel_mux

### no Nav2 (semi auto):

ros2 launch diffbot_sim sim.launch.py use_nav2:=false

## Terminal 2 - leaderbot

ros2 launch diffbot_sim leader_spawn.launch.py

## leader control
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/leader/cmd_vel


## Terminal 4 — RViz (Nav2 View):

ros2 launch diffbot_sim rviz.launch.py


## Terminal 5 — Automated trajectories:

ros2 run diffbot_sim trajectory    # Fixed distance trajectory
ros2 run diffbot_sim trajectory8   # Infinity (8-shape) trajectory
ros2 run diffbot_sim trajectory90  # Sharp 90-degree turn



## Terminal 6 — Following Algorithms:

ros2 run follow_target_cpp woa_dwb_tfc  # WOA Controller
ros2 run follow_target_cpp acc          # ACC Controller
ros2 run follow_target_cpp pid_dwb_tfc  # PID Controller



## Terminal 7 — Data Analysis:

python3 analyze_log.py         # Print statistical summary to terminal
python3 plot_results.py        # Generate tracking performance graphs
python3 plot_results_world.py  # Generate world-space trajectory map

## Terminal 8  — GUI:
cd ~/ros2_ws
python3 src/robot_gui/robot_gui/main.py

