# KUKA LBR Cartesian Impedance Control  

A repository for controlling the KUKA lbr or med with cartesian impedance control.

<table>
    <tr>
        <th>ROS2 Distro</td>
        <th>Controller</td>
        <th>FRI library</th>
        <th>LBR Stack</th>
    </tr>
    <tr>
        <td>Humble</td>
        <td><a href='humble-controllers'><img src='https://github.com/lucabeber/effort_controller/actions/workflows/humble.yml/badge.svg'></a><br/> </td>
        <td><a href='humble-fri-library'><img src='https://github.com/lbr-stack/fri/actions/workflows/build.yaml/badge.svg?branch=fri-1.15'></a><br/> </td>
        <td><a href='humble-lbr-stack'><img src='https://github.com/idra-lab/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-22.04-fri-1.15.yml/badge.svg'></a><br/> </td>
    </tr>
    <td>Jazzy</td>
        <td><a href='jazzy-controllers'><img src='https://github.com/lucabeber/effort_controller/actions/workflows/jazzy.yml/badge.svg'></a><br/> </td>
        <td><a href='jazzy-fri-library'><img src='https://github.com/lbr-stack/fri/actions/workflows/build.yaml/badge.svg?branch=fri-1.15'></a><br/> </td>
        <td>not tested</a><br/> </td>
    </tr>
</table>

## Setup
### Set the FRI impedance gains to zero 
Follows the instructions reported in the lbr stack documentation that you can find [here](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_fri_ros2_stack/doc/hardware_setup.html).  
Set the stiffness of the `JointImpedanceControlMode` in the `LBRServer.java` to zero for each joint, i.e.:
```
control_mode_ = new JointImpedanceControlMode(0, 0, 0, 0, 0, 0, 0);
```
Then synchronize the application to update the changes.
  
### Install
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

## Run the controllers

### Gravity compensation
```
source install/setup.bash
ros2 launch lbr_bringup hardware.launch.py ctrl:=gravity_compensation
```

---

### Cartesian impedance control
```
source install/setup.bash
ros2 launch lbr_bringup hardware.launch.py ctrl:=cartesian_impedance_controller
```
<div align="center">
<img src='https://github.com/idra-lab/kuka_impedance/blob/main/assets/cart_impedance.gif' width="640"/>
</div>
---


### Cartesian impedance control with nullspace task
This controller takes the initial configuration as the reference position for the nullspace control.  
Can be activated by setting
```
...
nullspace_stiffness: 10.0
...
```
in `lbr_fri_ros2_stack/lbr_description/ros2_control/lbr_controllers.yaml` in the `cartesian_impedance_controller` configuration.  
Then run the controller
```
source install/setup.bash
ros2 launch lbr_bringup hardware.launch.py ctrl:=cartesian_impedance_controller
```
<div align="center">
<img src='https://github.com/idra-lab/kuka_impedance/blob/main/assets/null_space_impedance.gif' width="640"/>
</div>

## References
- [Cartesian controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers.git)
- [lbr ROS2 stack](https://github.com/lbr-stack/lbr_fri_ros2_stack)
