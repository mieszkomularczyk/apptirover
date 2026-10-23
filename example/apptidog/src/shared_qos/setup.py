from setuptools import setup, find_packages

package_name = 'shared_qos'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your-email@example.com',
    description='RA ROS 2 package providing shared QoS profiles for publishers and subscribers.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    },
)
