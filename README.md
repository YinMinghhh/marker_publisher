# Brief
Tested on Ubuntu-20.04(Windows 10, WSL2) with ROS Noetic.  

## Install
Assuming that you have installed [ros_motion_planning](https://github.com/ai-winter/ros_motion_planning).  

```bash
cd ~/catkin_ws/src
git clone https://github.com/YinMinghhh/marker_publisher.git
cd ~/catkin_ws
catkin_make
```

## Usage
Bash 1
```bash
/your-path-to/ros_motion_planning/scripts/main.sh
```

Bash 2
```bash
source ~/catkin_ws/devel/setup.bash
rosrun marker_publisher marker_publisher_node
```

Bash 3
```bash
source ~/catkin_ws/devel/setup.bash
rosrun marker_publisher collision_checker_node
```

