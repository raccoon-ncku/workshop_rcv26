# Inside ch1_basics/launch/ch1_exercise.launch.py
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

included_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('rccn_kuka_robot_cell'),
            'launch',
            'robot.launch.py'
        ])
    ]),
    launch_arguments={'mode': 'manual'}.items()
)