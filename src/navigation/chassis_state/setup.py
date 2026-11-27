from glob import glob
from setuptools import setup

setup(
    name='chassis_state', version='0.1.0', packages=['chassis_state'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/chassis_state']),
                ('share/chassis_state', ['package.xml']),
                ('share/chassis_state/launch', glob('launch/*.launch.py')),
                ('share/chassis_state/config', glob('config/*.yaml'))],
    install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
    maintainer='Wanqiq', maintainer_email='robot@example.com',
    description='Measurement-aligned physical chassis state for yaw-mounted LIO',
    license='Apache-2.0',
    entry_points={'console_scripts': ['chassis_state = chassis_state.node:main',
                       'simulation_feedback = chassis_state.simulation_feedback:main']},
)
