from setuptools import setup

package_name = 'sensor_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adhithiya',
    maintainer_email='adhithiyashanmugarajan@gmail.com',
    description='A ROS 2 node for communicating with a TCP sensor.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = sensor_node.sensor_node:main',
        ],
    },
)
