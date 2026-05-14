"""
detect.launch.py - hele fase-1 pipelinen: kamera + HSV-deteksjon.

Inkluderer camera.launch.py (usb_cam -> /image_raw) og starter
hsv_detector som leser HSV-terskler fra pakkens installerte
config-katalog. Én kommando bringer opp hele deteksjonskjeden.

Opphav per blokk:
  - IncludeLaunchDescription + get_package_share_directory: standard
    ROS2-launchmoenster (Jazzy launch-tutorial).
  - Node()-konfig: samme moenster som camera.launch.py / Lab4.
  - Sammensetningen (kamera + detektor i én launch): generert.

Bruk:
    ros2 launch vision detect.launch.py
    # valgfritt: overstyr min_area
    ros2 launch vision detect.launch.py min_area:=800
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('vision')

    # Launch-argument saa terskel-stoerrelsen kan justeres uten
    # kodeendring (ROS2-kategorien "konfigurerbart"). Generert.
    min_area_arg = DeclareLaunchArgument(
        'min_area', default_value='500',
        description='Minste konturareal (px^2) for aa telle som kube.')

    # Inkluder kamera-bringup slik den allerede er definert.
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'camera.launch.py')))

    # Terskelfila ligger i pakkens delte config-katalog etter build.
    thresholds_file = os.path.join(
        pkg_share, 'config', 'hsv_thresholds.yaml')

    detector_node = Node(
        package='vision',
        executable='hsv_detector',
        name='hsv_detector',
        output='screen',
        parameters=[{
            'thresholds_file': thresholds_file,
            'min_area': LaunchConfiguration('min_area'),
        }],
    )

    return LaunchDescription([
        min_area_arg,
        camera_launch,
        detector_node,
    ])
