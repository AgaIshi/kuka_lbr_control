# kuka_impedance control
A repository for controlling the KUKA lbr or med with cartesian impedance control.


### Setup
#### Set the FRI impedance gains to zero 
Follows the instructions reported in the lbr stack documentation that you can find [here](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_fri_ros2_stack/doc/hardware_setup.html).  
Set the stiffness of the `JointImpedanceControlMode` in the `LBRServer.java` to zero for each joint, i.e.:
```
control_mode_ = new JointImpedanceControlMode(0, 0, 0, 0, 0, 0, 0);
```
Then synchronize the application to update the changes.
  
#### Install
Clone this repo inside your ros2 workspace as the `src` folder
```
git clone --recursive git@github.com:idra-lab/kuka_impedance.git src
```
Install ros2 dependencies
```
rosdep install --from-paths src -i -r -y
```
Build the workspace
```
colcon build --symlink-install
```

### Run the controllers

#### Gravity compensation
```
source install/setup.bash
ros2 launch lbr_bringup hardware.launch.py ctrl:=gravity_compensation
```

#### Cartesian impedance control
```
source install/setup.bash
ros2 launch lbr_bringup hardware.launch.py ctrl:=gravity_compensation
```
---

#### References
- [Cartesian controllers](git@github.com:fzi-forschungszentrum-informatik/cartesian_controllers.git)
- [lbr ROS2 stack](https://github.com/lbr-stack/lbr_fri_ros2_stack)