from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 核心控制节点 (包含 EKF、地形估计、FSM、动力学与平衡控制)
        Node(
            package='robot_control',
            executable='mobed_control_node',
            name='mobed_control_node',
            output='screen',
            emulate_tty=True,  # 保证日志颜色输出
            parameters=[
                # 这里可以添加实车参数配置
            ]
        ),
    ])
