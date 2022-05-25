from setuptools import setup

import os
from glob import glob


package_name = 'ROS2_SU065_D4380_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yokoyamayy',
    maintainer_email='yokowaiwai5757yy@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rs422node = ROS2_SU065_D4380_driver.comnode:main',
            'rs422_paramnode = ROS2_SU065_D4380_driver.paramnode:main',
            'rs422_keynode = ROS2_SU065_D4380_driver.paramkey:main',
            'robocon = ROS2_SU065_D4380_driver.harvestx_odm:main',
            'joycon = ROS2_SU065_D4380_driver.joycon_node:main'    
        ],
    },
)
