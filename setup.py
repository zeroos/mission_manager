from setuptools import find_packages
from setuptools import setup

package_name = 'mission_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Michal Barcis',
    author_email='michal.barcis@aau.at',
    maintainer='Michal Barcis',
    maintainer_email='michal.barcis@aau.at',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='TODO',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cli = mission_manager.mission_manager:cli',
            'cmd = mission_manager.mission_manager:cmd',
            'example_client = mission_manager.examples.mock_mission_client.mock_mission_client:start_mock_client',
            'example_auto_mission_manager = mission_manager.examples.auto_mission_manager.auto_mission_manager:start_auto_mission_manager',
            'example_fsm_client = mission_manager.examples.auto_mission_manager.fsm_client:start_fsm_client',
            'example_custom_cmd_client = mission_manager.examples.custom_command.custom_command:start_custom_cmd_client',
            'example_custom_cmd_manager = mission_manager.examples.custom_command.custom_command_amm:start_auto_mission_manager',
        ],
    },
)
