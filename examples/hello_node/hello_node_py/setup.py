from setuptools import find_packages, setup

package_name = 'hello_node_py'

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
    maintainer='Ioannis Selinis',
    maintainer_email='selinis.g@gmail.com',
    description='Minimal (Hello) ROS2 py node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "hello_node = hello_node_py.hello_node:main"
        ],
    },
)
