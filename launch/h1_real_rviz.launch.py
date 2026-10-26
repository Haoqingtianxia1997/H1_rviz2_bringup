from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=_setup),
    ])


def _setup(context, *args, **kwargs):
    pkg_share = FindPackageShare("h1_bringup").perform(context)
    urdf_file = PathJoinSubstitution([pkg_share, "model", "h1", "urdf", "h1_rviz.urdf"]).perform(context)
    with open(urdf_file, "r") as f:
        robot_description = f.read()

    bridge_script = PathJoinSubstitution([pkg_share, "scripts", "lowstate_to_joint_states.py"]).perform(context)
    joint_map_yaml = PathJoinSubstitution([pkg_share, "config", "h1_joint_map.yaml"]).perform(context)

    # ---- bridge command (NO extra space after ':=') ----
    bridge_cmd = [
        "python3", bridge_script,
        "--ros-args",
        "-p", f"lowstate_topic:=/lowstate",
        "-p", f"joint_states_topic:=/joint_states",
        "-p", f"joint_map_yaml:={joint_map_yaml}",
    ]

    return [
        # 1) /lowstate -> /joint_states
        ExecuteProcess(cmd=bridge_cmd, output="screen"),

        # 2) robot_state_publisher: URDF + /joint_states -> /tf
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),

        # 3) RViz
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
        ),

        # 4) Static transform publisher: camera_frame -> zed_camera_link
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_to_zed_static_transform",
            arguments=["0", "0", "0", "0", "0", "0", "camera_frame", "zed_camera_link"],
            output="screen",
        ),
    ]
    

