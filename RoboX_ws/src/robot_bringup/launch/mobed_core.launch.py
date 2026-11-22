import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hardware_config = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        'mobed_hardware.yaml',
    )

    launch_ecat_arg = DeclareLaunchArgument(
        'launch_ecat',
        default_value='false',
        description='Whether to also start EcatV2_Master soem_backend (requires root / RT privileges)',
    )
    launch_hub_arg = DeclareLaunchArgument(
        'launch_hub',
        default_value='true',
        description='Whether to start robox_hub_controller (SBUS teleop & command routing)',
    )
    ecat_interface_arg = DeclareLaunchArgument(
        'ecat_interface',
        default_value='enp2s0',
        description='EtherCAT NIC name for soem_backend',
    )
    ecat_config_arg = DeclareLaunchArgument(
        'ecat_config',
        default_value='',
        description='Path to generated EcatV2_Master config.yaml when launch_ecat:=true',
    )
    rt_cpu_arg = DeclareLaunchArgument(
        'rt_cpu',
        default_value='0',
        description='Isolated CPU core for soem_backend RT loop',
    )
    non_rt_cpus_arg = DeclareLaunchArgument(
        'non_rt_cpus',
        default_value='1-7',
        description='Non-RT CPU cores for soem_backend',
    )

    return LaunchDescription([
        launch_ecat_arg,
        launch_hub_arg,
        ecat_interface_arg,
        ecat_config_arg,
        rt_cpu_arg,
        non_rt_cpus_arg,

        # 可选：同时拉起 EcatV2_Master EtherCAT 主站节点（需在 root 下运行）
        Node(
            package='soem_wrapper',
            executable='soem_backend',
            name='soem_backend',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('launch_ecat')),
            parameters=[{
                'interface': LaunchConfiguration('ecat_interface'),
                'rt_cpu': LaunchConfiguration('rt_cpu'),
                'non_rt_cpus': LaunchConfiguration('non_rt_cpus'),
                'config_file': LaunchConfiguration('ecat_config'),
            }],
        ),

        # 上层 Hub 控制器 (SBUS 遥控器解析 + 速度平滑 -> RoboxChassisControl)
        Node(
            package='hub_controller',
            executable='robox_hub_controller',
            name='robox_hub_controller',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('launch_hub')),
            parameters=[hardware_config],
        ),

        # 底层底盘控制节点 (直连 EcatV2_Master 12 个实车电机 + HiPNUC CH010 IMU)
        Node(
            package='chassis_controllers',
            executable='dm_lk_robox_chassis_controller',
            name='dm_lk_robox_chassis_controller',
            output='screen',
            emulate_tty=True,
            parameters=[hardware_config],
        ),
    ])
