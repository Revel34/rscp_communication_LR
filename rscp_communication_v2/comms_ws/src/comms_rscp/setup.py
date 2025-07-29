from setuptools import find_packages, setup

package_name = 'comms_rscp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'pyserial',    # serial port I/O
        'evdev',       # USB/input-event reading
        'cobs',          # COBS framing for RSCP
        'rscp-protobuf', # generated protobuf bindings
                      ],
    zip_safe=True,
    maintainer='revel34',
    maintainer_email='revel34@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener_node = comms_rscp.RSCP:main',
        ],
    },
)
