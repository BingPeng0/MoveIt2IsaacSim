import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import launch_ros
from launch_param_builder import ParameterBuilder

def generate_launch_description():

    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="isaac",
        description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",)

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="dual_arm_panda_isaac", package_name="dual_arm_panda_isaac_moveit_config")
        .robot_description(
            #file_path="config/panda.urdf.xacro",
            file_path="config/panda.u_servo.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type")},)
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory("moveit2_tutorials")
            + "/config/motion_planning_python_api_tutorial.yaml")
        .to_moveit_configs())

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],)

    # RViz
    rviz_config_file = os.path.join(
        get_package_share_directory("moveit2_tutorials"),
        "config",
        "panda_moveit_config.rviz",)

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,],)

    # Static TF
    world2robot_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "left_panda_link0"],)
    
    hand2camera_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=[
            "0.04",
            "0.0",
            "0.04",
            "0.0",
            "0.0",
            "0.0",
            "left_panda_hand",
            "left_sim_camera",],)

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],)

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("dual_arm_panda_isaac_moveit_config"),
        "config",
        "ros2_controllers.yaml",)
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",)

    # Load controllers
    load_controllers = []
    for controller in [
        "joint_state_broadcaster",
        "left_arm_controller",
        "right_arm_controller",
        # "left_hand_controller",
        # "right_hand_controller"
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",)]
    
        # Get parameters for the Servo node
    servo_params_left = {
        "moveit_servo": ParameterBuilder("moveit_servo")
        .yaml("config/panda_simulated_config_left.yaml")
        .to_dict()}
    servo_params_right = {
        "moveit_servo": ParameterBuilder("moveit_servo")
        .yaml("config/panda_simulated_config_right.yaml")
        .to_dict()}

    # This sets the update rate and planning group name for the acceleration limiting filter.
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_left = {"planning_group_name": "left_panda_arm"}
    planning_group_right = {"planning_group_name": "right_panda_arm"}

    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node_left = launch_ros.actions.Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node_left",
        parameters=[
            servo_params_left,
            acceleration_filter_update_period,
            planning_group_left,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",)
    
    servo_node_right = launch_ros.actions.Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node_right",
        parameters=[
            servo_params_right,
            acceleration_filter_update_period,
            planning_group_right,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",)
    
    dual_arm_joint_commands_filter = Node(
        package='moveit2_tutorials',
        executable='dual_arm_joint_commands_filter.py',
        name='dual_arm_joint_commands_filter',
        output='screen',
        parameters=[],)

    return LaunchDescription(
        [            
            ros2_control_hardware_type,
            rviz_node,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,           
            dual_arm_joint_commands_filter,
            servo_node_left,
            servo_node_right
        ]
        + load_controllers
    )
