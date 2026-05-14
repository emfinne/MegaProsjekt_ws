# Opphav: direkte mal fra Lab4 src/camera_pipeline/setup.py (AIS2105).
# Endringer: package_name, maintainer-felt, license, og rensket
# entry_points-blokken (noder legges til etterhvert).
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kristian Loekkeberg',
    maintainer_email='kristian.lokkeberg@gmail.com',
    description='Maskinsyn-modul for UR-prosjektet i AIS2105 (kamera-bringup, HSV-deteksjon, logging).',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hsv_tuner = vision.hsv_tuner:main',
            'hsv_detector = vision.hsv_detector:main',
            # 'detection_logger = vision.detection_logger:main',  # neste
        ],
    },
)
