# MHE_ACADO_ROS2


## Run ROS-Bridge
```bash
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```


## in ROS1 run 
```bash
roslaunch bluerov2_trajectory test.launch 
```

```bash
roslaunch bluerov2_nmpc bluerov2_nmpc.launch 
```



## in ROS2

```bash
source /opt/ros/foxy/setup.bash
source install/local_setup.sh
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```



