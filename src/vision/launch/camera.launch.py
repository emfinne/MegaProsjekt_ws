"""
camera.launch.py — bringer opp Logitech C920 via usb_cam.

Opphav per blokk:
  - LaunchDescription + Node-moenster: Lab4 launch/pipeline.launch.py
  - PathJoinSubstitution + FindPackageShare for parameter-fil:
    standard ROS2-moenster (Jazzy-launchtutorial)
  - CAMERA_BY_ID-konstanten: generert fra `ls /dev/v4l/by-id/` paa
    Kristians maskin 2026-05-12 (serienummer 942F52FF)
  - resolve_camera_device(): generert som workaround for usb_cam 0.8.1
    som ikke foelger symlinks (oppdaget under testing — den behandlet
    "../../video4" bokstavelig og krasjet med "Device not available")

Bruk:
    ros2 launch vision camera.launch.py

Verifiser med:
    ros2 topic hz /image_raw
    ros2 run rqt_image_view rqt_image_view /image_raw
"""
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


# Stabil identifier basert paa serienummer i USB-deskriptoren. Endres
# kun hvis kameraet erstattes med en annen fysisk enhet. Generert.
CAMERA_BY_ID = (
    '/dev/v4l/by-id/'
    'usb-046d_HD_Pro_Webcam_C920_942F52FF-video-index0'
)


def resolve_camera_device() -> str:
    """
    Resolverer by-id-symlinken til faktisk /dev/videoN.
    Faller tilbake til /dev/video2 hvis symlinken ikke finnes (f.eks.
    naar kameraet ikke er tilkoblet) saa launch ikke krasjer foer
    usb_cam-noden faar logge "device not available".

    Generert som workaround for usb_cam 0.8.1 sin manglende symlink-
    haandtering. Ingen ekstern kilde.
    """
    if os.path.islink(CAMERA_BY_ID):
        return os.path.realpath(CAMERA_BY_ID)
    return '/dev/video2'


def generate_launch_description():

    # Pattern fra Lab4 pipeline.launch.py — peker paa pakkens delte
    # config-katalog via FindPackageShare.
    camera_params = PathJoinSubstitution([
        FindPackageShare('vision'),
        'config',
        'camera_params.yaml',
    ])

    # Node-strukturen er fra Lab4. Overriden av video_device i andre
    # parameters-element er generert (haandterer symlink-bugen over).
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[
            camera_params,
            {'video_device': resolve_camera_device()},
        ],
    )

    return LaunchDescription([
        usb_cam_node,
    ])
