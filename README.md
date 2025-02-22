# acceleration_robotics_task

# Building Nav2, TurtleBot3, and Related Packages from Source

## Prerequisites
Ensure you have **Ubuntu 22.04** with **ROS 2 Humble** installed. If not, install ROS 2 Humble:
```bash
sudo apt update && sudo apt install -y ros-humble-desktop -y
```
Also, install necessary dependencies:
```bash
sudo apt install -y python3-colcon-common-extensions python3-vcstool git
```

## Task 1: Set Up and Build the Repositories

### 1. Create the Workspace
```bash
mkdir -p ~/nav2_ws/src
cd ~/nav2_ws
```

### 2. Clone the Repositories

#### Clone Navigation2 (Nav2)
```bash
cd ~/nav2_ws/src
git clone -b humble https://github.com/ros-planning/navigation2.git
vcs import < navigation2/tools/ros2_dependencies.repos
```

#### Clone TurtleBot3 Packages
```bash
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
```
> [!IMPORTANT]
> Remember to add and replace the directories in the official cloned repository with the ones here.
That includes replacing turtlebot3_simulations in its entirety, adding turtlebot_pose_tracker to the src whole, and replacing turtlebot3_teleop
and turtlebot3_cartographer from within the turtlebot3 official repository.

#### Clone TurtleBot3 Messages
```bash
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
```

#### Clone TurtleBot3 Simulations
```bash
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

### 3. Install Dependencies
```bash
cd ~/nav2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build the Workspace
```bash
cd ~/nav2_ws
colcon build --symlink-install
```

### 5. Source the Workspace
```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
source ~/nav2_ws/install/setup.bash
source /usr/share/gazebo/setup.sh
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export TURTLEBOT3_MODEL=waffle
export ROS_DOMAIN_ID=30 #TURTLEBOT3
```
To make it persistent:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/turtlebot3_ws/install/setup.bash" >> ~/.bashrc
echo "source ~/nav2_ws/install/setup.bash" >> ~/.bashrc
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=30 #TURTLEBOT3" >> ~/.bashrc
source ~/.bashrc
```

### 6. Verify Installation
#### Check Nav2 Packages
```bash
ros2 pkg list | grep nav2
```

#### Check TurtleBot3 Packages
```bash
ros2 pkg list | grep turtlebot3
```

#### Check TurtleBot3 World
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## Task 2: Launch Custom World

We have modified the empty world to include two static obstacles and one dynamic obstacle. To launch this custom world, follow these steps:

```bash
source ~/.bashrc
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

## Task 3: Mapping with Cartographer

To create a map of the environment using **Cartographer**, follow these steps:

1. Source the environment:
```bash
source ~/.bashrc
```
2. Launch Cartographer:
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py
```
3. Open another terminal, source the environment again, and run the teleoperation node to move the robot around:
```bash
source ~/.bashrc
ros2 run turtlebot3_teleop teleop_keyboard
```
4. Navigate the robot around the environment to generate the map.
5. Once a stable map is visible in **RViz**, open a new terminal, source the environment, and save the map:
```bash
source ~/.bashrc
ros2 run nav2_map_server map_saver_cli -f ~/map
```
This will generate a `map.yaml` file in the home directory, which can be used for navigation.

## Task 4: Using the Saved Map for Navigation

1. Ensure the saved map is being published correctly.
2. Open a new terminal, source the environment, and bring up the robot:
```bash
source ~/.bashrc
ros2 launch turtlebot3_bringup robot.launch.py
```
3. Open another terminal, source the environment again, and launch **Navigation2** with the saved map:
```bash
source ~/.bashrc
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml
```
4. Open a new terminal, source the environment, and start the pose tracker:
```bash
source ~/.bashrc
ros2 launch turtlebot_pose_tracker turtlebot_pose_tracker.launch.py
```

## Task 5: Tuning the Planner
One can go to `turtlebot3/turtlebot3_navigation2/param/` and alter the YAML files for tuning the planner.

## Task 6: Multi-Point Navigation
One can use the example in the Nav2 repository for traversing multiple points in `nav2_simple_commander` using `navigate_through_poses` or the `turtlebot3_example` using `turtlebot3_position_control`. 
