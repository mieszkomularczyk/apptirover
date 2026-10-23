from setuptools import setup
from glob import glob

package_name = 'follower'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maciej Kaca',
    maintainer_email='maciej.kaca@apptimia.com',
    description='Follower for Go2 Robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follower_node = follower.follower_node:main',
            'target_tracker_node = follower.target_tracker_node:main',
        ],
    },
)
