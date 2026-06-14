import os
from glob import glob
from setuptools import setup

package_name = 'rscp_mission_orchestrator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='revel34',
    maintainer_email='you@example.com',
    description='Mission orchestrator for the RSCP rover stack.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'mission_orchestrator = rscp_mission_orchestrator.mission_orchestrator:main',
        ],
    },
)