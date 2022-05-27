"""Setup."""
from setuptools import setup

from glob import glob

package_name = 'su065d4380_v1'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='m12watanabe1a',
    maintainer_email='m12watanabe1a@gmail.com',
    description='HarvestX SU065 D4380 Driver V1',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            '{node_name} = {pkg_path}.{node_name}:main'.format(
                pkg_path=package_name,
                node_name='comnode',),
            '{node_name} = {pkg_path}.{node_name}:main'.format(
                pkg_path=package_name,
                node_name='paramnode',),
            '{node_name} = {pkg_path}.{node_name}:main'.format(
                pkg_path=package_name,
                node_name='paramkey',),
            '{node_name} = {pkg_path}.{node_name}:main'.format(
                pkg_path=package_name,
                node_name='odomnode',),
            '{node_name} = {pkg_path}.{node_name}:main'.format(
                pkg_path=package_name,
                node_name='joynode',),
        ],
    },
)
