import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

# Optional launch arguments that override navigation.yaml for one run, e.g.
#   perception:=blue_aware footprint_check:=true steering:=center speed:=0.25 max_yaw:=2.0
# Left out = the value in navigation.yaml.
OVERRIDES = {
    'perception': 'perception',
    'steering': 'steering',
    'footprint_check': 'footprint_check',
    'speed': 'requested_forward_speed_m_s',
    'max_yaw': 'max_yaw_rate_rad_s',
    'yellow_gap': 'path_jump_px',  # perception: yellow
    'blue_aware_gap': 'max_gap_px',  # perception: blue_aware
    'explore': 'explore_when_blocked',
}


def navigation_node(context):
    config = str(Path(get_package_share_directory('team_navigation')) /
                 'config' / 'navigation.yaml')
    overrides = {}
    for argument, parameter in OVERRIDES.items():
        text = LaunchConfiguration(argument).perform(context)
        if text:
            value = yaml.safe_load(text)  # 'true' -> True, '0.25' -> 0.25
            if parameter in ('requested_forward_speed_m_s', 'max_yaw_rate_rad_s', 'max_gap_px'):
                value = float(value)
            overrides[parameter] = value
    return [Node(package='team_navigation', executable='navigation',
                 namespace=LaunchConfiguration('namespace'), output='screen',
                 parameters=[config, overrides])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', description='Assigned robot namespace'),
        *(DeclareLaunchArgument(name, default_value='',
                                description=f'Override {parameter} (blank = navigation.yaml)')
          for name, parameter in OVERRIDES.items()),
        OpaqueFunction(function=navigation_node),
    ])
