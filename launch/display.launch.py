import os

import launch
import launch.actions
import launch.substitutions
import launch_ros
import launch_ros.actions
import launch_ros.descriptions
import launch_ros.substitutions

def generate_launch_description():

    pkg = launch_ros.substitutions.FindPackageShare(package='x2_description').find('x2_description')

    urdf_file = launch.actions.DeclareLaunchArgument(
        'urdf',
        default_value='x2_ultra.urdf',
        description='URDF file name inside the urdf/ directory of x2_description',
    )

    # ref:https://answers.ros.org/question/417369/caught-exception-in-launch-see-debug-for-traceback-unable-to-parse-the-value-of-parameter-robot_description-as-yaml/
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': launch_ros.descriptions.ParameterValue(
                launch.substitutions.Command(
                    ['cat ', os.path.join(pkg, 'urdf/'), launch.substitutions.LaunchConfiguration('urdf')]
                ),
                value_type=str,
            )
        }]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(pkg, 'launch/config.rviz')]
    )

    return launch.LaunchDescription([
        urdf_file,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])
