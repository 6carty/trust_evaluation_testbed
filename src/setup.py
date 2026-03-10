from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'trust_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
     
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='irlab',
    maintainer_email='irlab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reliability_service = scripts.reliability_service:main',
            'goal_setting_service = trust_sim.trust_sim.goal_setting_service:main',
            'goal_monitor = trust_sim.trust_sim.goal_monitor:main',
            'comfort_service = trust_sim.comfort_service:main',
            'test = trust_sim.test:main',
        ],
    },
)
