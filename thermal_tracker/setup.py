from setuptools import setup
from glob import glob
import os

package_name = 'thermal_tracker'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # use direct package reference, not find_packages
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include message definitions so rosidl can find them
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='caleb.chadwick1@icloud.com',
    description='Thermal camera target detection node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # format: "executable_name = package_name.file_name:function"
            'thermal_detector = thermal_tracker.thermal_detector:main',
        ],
    },
)

