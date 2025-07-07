# KUKA LBR Control  

A repository for controlling the KUKA lbr or med with cartesian impedance control using ROS2.

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
        <td><a href='jazzy-lbr-stack'><img src='https://github.com/idra-lab/lbr_fri_ros2_stack/actions/workflows/build-ubuntu-24.04-fri-1.15.yml/badge.svg'></a><br/></td>
    </tr>
</table>

### Install
The installation guide is available in the [Wiki page](https://github.com/idra-lab/kuka_lbr_control/wiki)

## Run the controllers on real hardware
### Kinematics control
TODO
### Gravity compensation
```
source install/setup.bash
ros2 launch lbr_bringup hardware.launch.py ctrl:=gravity_compensation
```

---

### Cartesian impedance control
<div align="center">
<img src='https://github.com/idra-lab/kuka_impedance/blob/main/assets/cart_impedance.gif' width="640"/>
</div>

---  

### Cartesian impedance control with nullspace task
<div align="center">
<img src='https://github.com/idra-lab/kuka_impedance/blob/main/assets/null_space_impedance.gif' width="640"/>
</div>
---  

## Test the controllers in Gazebo
<img src='https://github.com/idra-lab/kuka_impedance/blob/main/assets/gazebo.gif' width="640"/>
</div>

## Tracking a reference trajectory
This controller takes a trajectory as input a `PoseStamped` message and tracks it.
You need to publish a `PoseStamped` message on the `/lbr/target_frame` topic.  
Some examples trajectories are already implemented [here](https://github.com/Hydran00/controller_evaluation) where tracking performances are also visualized.
```
git clone git@github.com:Hydran00/controller_evaluation.git
cd controller_evaluation/
```
Make sure that the robot type is set to `"kuka"` in the `initialize.py` file. 
Adjust the trajectory parameters in the `traj_sin.py` or `traj_lin.py` file. 
Then run the trajectory publisher:
```
source /opt/ros/humble/setup.bash
python3 traj_sin.py # traj_lin.py
```

## References
- [Cartesian controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers.git)
- [lbr ROS2 stack (KUKA Hardware Interface)](https://github.com/lbr-stack/lbr_fri_ros2_stack)
