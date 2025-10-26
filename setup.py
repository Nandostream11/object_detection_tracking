from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'object_detection_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anandv',
    maintainer_email='anandvk113@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector = object_detection_tracking.object_detection:main',
            'object_tracker = object_detection_tracking.object_tracking:main',
            'camera_publisher = object_detection_tracking.cam_data:main',
            'apriltag_detector = object_detection_tracking.April_tag_det:main',
        ],
    },
)
