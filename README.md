# UofL-F1Tenth
Spring 2025 Capstone Project files

## How to run

1. Follow install instructions at [F1Tenth Autoware Universe](https://github.com/autowarefoundation/autoware_universe/tree/f1tenth_galactic/f1tenth)

2. Clone this repo and run in the root directory of the repo
    
```bash
source /opt/ros/galactic/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro galactic -r
colcon build --symlink-install
source install/setup.bash
ros2 run <package_name> <node_name>
```

3. Run the simulation
```bash
source /opt/ros/galactic/setup.bash
cd ~/autoware && . install/setup.bash
ros2 launch launch_autoware_f1tenth demo_launch.py
```