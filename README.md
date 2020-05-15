Attach iris drone to an object in gazebo

## Build

```
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
git clone https://github.com/Open-UAV/gazebo_ros_link_attacher.git
cd ..
catkin build
source devel/setup.bash
```

## Run

`<plugin name="ros_link_attacher_plugin" filename="libgazebo_ros_link_attacher.so"/>`

Add the above lines to the gazebo world file to enable link attach plugin.
    

`rosrun gazebo_ros_link_attacher attach.py`

The above rosrun command will create a rostopic `/attach` where 

    if you send string "ATTACH", the iris drone will attach to the sample_probe object in the world. 
    if you send string "DETACH", the iris drone will detach the sample_probe object. 

The attach happens if the drone is with 30 degrees from the object and is less than 0.6 meters.


---
Code modified from 
    https://github.com/pal-robotics/gazebo_ros_link_attacher
