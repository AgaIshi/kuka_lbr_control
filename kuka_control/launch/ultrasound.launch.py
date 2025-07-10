from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from typing import Dict, Optional, Union
from lbr_bringup.description import LBRDescriptionMixin
from lbr_bringup.ros2_control import LBRROS2ControlMixin
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
import xml.etree.ElementTree as ET


def my_param_robot_description(
    model: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
        "model", default="iiwa7"
    ),
    robot_name: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
        "robot_name", default="lbr"
    ),
    mode: Optional[Union[LaunchConfiguration, bool]] = LaunchConfiguration(
        "mode", default="mock"
    ),
    system_config_path: Optional[
        Union[LaunchConfiguration, str]
    ] = PathJoinSubstitution(
        [
            FindPackageShare(
                LaunchConfiguration("sys_cfg_pkg", default="kuka_control")
            ),
            LaunchConfiguration(
                "sys_cfg", default="config/lbr_system_config_position.yaml"
            ),
        ]
    ),
) -> Dict[str, str]:
    robot_description = {
        "robot_description": Command(
            [
                FindExecutable(name="xacro"),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("kuka_control"),
                        "urdf/iiwa14_ultrasound.xacro",
                    ]
                ),
                " robot_name:=",
                robot_name,
                " mode:=",
                mode,
                " system_config_path:=",
                system_config_path,
            ]
        )
    }
    return robot_description

def node_ros2_control(
    robot_name: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
        "robot_name", default="lbr"
    ),
    use_sim_time: Optional[Union[LaunchConfiguration, bool]] = LaunchConfiguration(
        "use_sim_time", default="false"
    ),
    robot_description: Optional[Dict[str, str]] = {},
    **kwargs,
) -> Node:
    return Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"use_sim_time": use_sim_time},
            PathJoinSubstitution(
                [
                    FindPackageShare(
                        LaunchConfiguration("ctrl_cfg_pkg", default="kuka_control")
                    ),
                    LaunchConfiguration("ctrl_cfg", default="config/controllers.yaml"),
                ]
            ),
            robot_description,
        ],
        namespace=robot_name,
        remappings=[
            ("~/robot_description", "robot_description"),
            ("cartesian_impedance_controller/target_frame", "target_frame"),
            ("cartesian_impedance_controller/target_wrench", "target_wrench"),
            ("joint_impedance_controller/target_frame", "target_frame"),
            ("joint_impedance_controller/target_wrench", "target_wrench"),
            ("kuka_clik_controller/target_frame", "target_frame"),
        ],
        **kwargs,
    )


def apply_transparency(robot_description_string):
    # Parse the robot description XML string
    root = ET.fromstring(robot_description_string)

    # Iterate over all visual elements
    for visual in root.findall(".//visual"):
        parent_link = visual.getparent() if hasattr(visual, "getparent") else None
        # ElementTree doesn't support getparent(), so we get parent by searching
        # workaround: find the link that contains this visual

        # Find the parent link by iterating links (since ElementTree has no getparent)
        parent_link_name = None
        for link in root.findall(".//link"):
            if visual in link.findall("visual"):
                parent_link_name = link.attrib.get("name")
                break

        if parent_link_name == "probe_visual":
            # Skip visuals belonging to "probe_visual" link
            continue

        material = visual.find('material')

        if material is None:
            # If no material tag, create one
            material = ET.SubElement(visual, 'material')
        # Set the material name to "transparent"
        material.set('name', 'transparent')

        # Remove existing color tags
        for color in material.findall('color'):
            material.remove(color)
        # Add the transparent color (RGBA) element
        color = ET.SubElement(material, 'color')
        color.set('rgba', '0.2 0.2 0.2 0.0')  # Adjust as needed

    # Convert back to string
    return ET.tostring(root, encoding='unicode')

def launch_setup(context, *args, **kwargs):
    ctrl = LaunchConfiguration("ctrl").perform(context)

    # Dynamically set sys_cfg based on ctrl value
    sys_cfg = "config/lbr_system_config_position.yaml"
    if (
        ctrl == "cartesian_impedance_controller"
        or ctrl == "joint_impedance_controller"
        or ctrl == "gravity_compensation"
    ):
        sys_cfg = "config/lbr_system_config_torque.yaml"

    # Build robot_description Command substitution
    robot_description_cmd = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("kuka_control"),
                    "urdf/iiwa14_ultrasound.xacro",
                ]
            ),
            " robot_name:=",
            LaunchConfiguration("robot_name"),
            " mode:=",
            "hardware",
            " system_config_path:=",
            PathJoinSubstitution(
                [FindPackageShare("kuka_control"), sys_cfg]
            ),
        ]
    )

    robot_description_str = robot_description_cmd.perform(context)
    # Apply transparency to the robot description string
    robot_description_str = apply_transparency(robot_description_str)
    robot_description = {"robot_description": robot_description_str}

    ros2_control_node = node_ros2_control(
        use_sim_time=False, robot_description=robot_description
    )

    robot_state_publisher = LBRROS2ControlMixin.node_robot_state_publisher(
        robot_description=robot_description, use_sim_time=False
    )

    joint_state_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
        controller="joint_state_broadcaster"
    )
    force_torque_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
        controller="force_torque_broadcaster"
    )
    lbr_state_broadcaster = LBRROS2ControlMixin.node_controller_spawner(
        controller="lbr_state_broadcaster"
    )
    controller = LBRROS2ControlMixin.node_controller_spawner(
        controller=LaunchConfiguration("ctrl")
    )

    controller_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                joint_state_broadcaster,
                force_torque_broadcaster,
                lbr_state_broadcaster,
                controller,
            ],
        )
    )

    return [
        robot_state_publisher,
        ros2_control_node,
        controller_event_handler,
    ]

def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # Basic args
    ld.add_action(LBRDescriptionMixin.arg_model())
    ld.add_action(LBRDescriptionMixin.arg_robot_name())
    ld.add_action(
        DeclareLaunchArgument(
            name="ctrl",
            default_value="kuka_clik_controller",
            description="Desired default controller.",
            choices=[
                "admittance_controller",
                "joint_trajectory_controller",
                "forward_position_controller",
                "lbr_joint_position_command_controller",
                "lbr_torque_command_controller",
                "lbr_wrench_command_controller",
                "twist_controller",
                "gravity_compensation",
                "cartesian_impedance_controller",
                "joint_impedance_controller",
                "kuka_clik_controller",
            ],
        )
    )

    # Opaque function to evaluate 'ctrl' and configure dependent args/nodes
    op = OpaqueFunction(function=launch_setup)
    ld.add_action(op)
    return ld
