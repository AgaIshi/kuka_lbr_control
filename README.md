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

## Install
The installation guide is available in the [Wiki page](https://github.com/idra-lab/kuka_lbr_control/wiki)

## Run the controllers on real hardware
### Kinematics controls
<div align="center">
<img src='https://github.com/idra-lab/kuka_impedance/blob/main/assets/videos/kin.gif' width="640"/>
</div>

---

### Gravity Compensation
<div align="center">
<img src='https://github.com/idra-lab/kuka_impedance/blob/main/assets/videos/grav.gif' width="640"/>
</div>

---

### Cartesian Impedance Control with Null Space Task
<div align="center">
<img src='https://github.com/idra-lab/kuka_impedance/blob/main/assets/videos/imp.gif' width="640"/>
</div>

---  

## Gazebo Simulation
<div align="center">
<img src='https://github.com/idra-lab/kuka_impedance/blob/main/assets/videos/gazebo.gif' width="640"/>
</div>

---
---  

## References
- [Cartesian controllers](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers.git)
- [lbr ROS2 stack (KUKA Hardware Interface)](https://github.com/lbr-stack/lbr_fri_ros2_stack)
