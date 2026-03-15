from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'decision_maker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phudh',
    maintainer_email='dohuuphu25@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "decision_maker_node = decision_maker.decision_maker_node:main",
            "text_command_node = decision_maker.text_command_node:main",
            'mock_nav_server = decision_maker.mock_nav_server:main',
            'mock_grasp_server = decision_maker.mock_grasp_server:main',
            "cancel_command_node = decision_maker.cancel_command_node:main",
            'nl_command_node = decision_maker.nl_command_node:main',
            'audio_command_node = decision_maker.audio_command_node:main',
            "decision_maker_test = decision_maker.decision_maker_test:main",
        ],
    },
)
