# UofL-F1Tenth
Spring 2025 Capstone Project files

## Building 

```bash
git clone https://github.com/blaketigges/uofl-f1tenth
cd uofl-f1tenth
rosdep install -y --from-paths src --ignore-src --rosdistro galactic -r
colcon build --symlink-install
```


## How to run

1. Follow install instructions at [F1Tenth Autoware Universe](https://github.com/autowarefoundation/autoware_universe/tree/f1tenth_galactic/f1tenth)

2. Run in the root directory of the repo
    
```bash
source /opt/ros/galactic/setup.bash
cd uofl-f1tenth && . install/setup.bash
ros2 run <package_name> <node_name> 
```

3. Run the simulation
```bash
source /opt/ros/galactic/setup.bash
cd ~/autoware && . install/setup.bash
ros2 launch launch_autoware_f1tenth demo_launch.py
```