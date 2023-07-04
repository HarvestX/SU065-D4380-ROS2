"""Setup."""

from glob import glob

from setuptools import setup

package_name = 'su065d4380_bringup'

setup(
    name=package_name,
    version='1.2.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/{}/launch'.format(package_name), glob('launch/*.launch.py')),
        ('share/{}/rviz'.format(package_name), glob('rviz/*.rviz')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='m12watanabe1a',
    maintainer_email='m12watanabe1a@gmail.com',
    description='HarvestX SU065 D4380 Driver bringup package.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
