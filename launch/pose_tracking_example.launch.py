import os
import launch #.conditions.IfCondition
import launch.actions
import launch_ros.actions 
import launch_param_builder 
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    
    # Arguments for use in further Launch API can be accessed this way, can be converted to concrete text with launch_rviz_lc.perform(context)
    launch_rviz_lc = launch.substitutions.LaunchConfiguration("launch_rviz")
    use_fake_hardware_lc = launch.substitutions.LaunchConfiguration("use_fake_hardware")

    # Already converted to context can be accessed this way
    ur_type = context.launch_configurations["ur_type"] 
    #robot_ip = context.launch_configurations["robot_ip"]
    
    init_pos_file = os.path.join(
        get_package_share_directory("servo_ur_pose_tracking_demo"),
        "config",
        "initial_positions.yaml",
    )

    ur_subs = dict(
        name="ur", 
        ur_type=ur_type,
        # robot_ip=robot_ip, #this shouldn't be needed even for real hardware/URSim, since driver is external
        use_fake_hardware="True", #again, this is just for Rviz and servo which don't use it
        initial_positions_file=init_pos_file
    )

    robot_description = launch_param_builder.ParameterBuilder("ur_description").xacro_parameter(
        parameter_name="robot_description",
        file_path="urdf/ur.urdf.xacro",
        mappings=ur_subs
    ).to_dict()
    
    robot_description_semantic = launch_param_builder.ParameterBuilder("ur_moveit_config").xacro_parameter(
        parameter_name="robot_description_semantic",
        file_path="srdf/ur.srdf.xacro",
        mappings=dict(name="ur")
    ).to_dict()
    
    servo_params = {
        "moveit_servo": launch_param_builder.ParameterBuilder("servo_ur_pose_tracking_demo")
        .yaml("config/pose_tracking_settings.yaml")
        .yaml("config/ur_simulated_config_pose_tracking.yaml")
        .to_dict()
    }

    # RViz
    rviz_config_file = (
        get_package_share_directory("moveit_servo")
        + "/config/demo_rviz_pose_tracking.rviz"
    )
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        # prefix=['xterm -e gdb -ex run --args'],
        output="log",
        arguments=["-d", rviz_config_file],
        condition=launch.conditions.IfCondition(launch_rviz_lc),
        parameters=[robot_description, robot_description_semantic], #could also just allow these to come in on topics, but probably faster here
    )

    # Publishes tf's for the robot
    robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        condition=launch.conditions.IfCondition(use_fake_hardware_lc),
    )

    # A node to publish world -> panda_link0 transform
    static_tf = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    pose_tracking_node = launch_ros.actions.Node(
        package="moveit_servo",
        executable="servo_pose_tracking_demo",
        #executable="servo_node_main",
        # prefix=['xterm -e gdb -ex run --args'],
        output="screen",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic
        ],
    )

    # ros2_control controllers file and update rate file
    ros2_controllers_path = os.path.join(
        get_package_share_directory("ur_robot_driver"),
        "config",
        "ur_controllers.yaml",
    )
    
    update_rate_config_file = os.path.join(
        get_package_share_directory("ur_robot_driver"),
        "config",
        f"{ur_type}_update_rate.yaml",
    )

    # if you want to use real hardware or URSim, launch the driver externally
    # we don't launch the ROS 2 control or controller nodes
    fake_ros2_control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, update_rate_config_file, ros2_controllers_path],
        output="screen",
        condition=launch.conditions.IfCondition(use_fake_hardware_lc),
    )

    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
        condition=launch.conditions.IfCondition(use_fake_hardware_lc),
    )

    arm_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "-c", "/controller_manager"],
        condition=launch.conditions.IfCondition(use_fake_hardware_lc),
    )

    return [
            rviz_node,
            static_tf,
            pose_tracking_node,
            fake_ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            robot_state_publisher,
    ]


def generate_launch_description():
    
    declared_args = []
    declared_args.append(
        launch.actions.DeclareLaunchArgument(
            "launch_rviz",
            default_value="True"
        )
    )
    declared_args.append(
        launch.actions.DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e"] #seems like the UR3 gets close to the base cyl singularity
        )
    )
    # declared_args.append(
    #     launch.actions.DeclareLaunchArgument(
    #         "robot_ip",
    #         default_value="192.168.56.101",
    #         description="IP address to use for URSim or real robot if use_fake_hardware is false (actually, maybe this is unnnecessary, since we don't launch ROS2 control)"
    #     )
    # )
    declared_args.append(
        launch.actions.DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="True",
            description="If false, skips launching ROS 2 control node and controller, assuming the driver is running and communicating with the robot already, and the YAML-configured controller is running."
        )
    )

    return launch.LaunchDescription(declared_args + [launch.actions.OpaqueFunction(function=launch_setup)])
