from setuptools import setup
from glob import glob

package_name = 'tracker'

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
    description='Tracking node using Ultralytics',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracker_node = tracker.tracker_node:main',
        ],
    },
)
