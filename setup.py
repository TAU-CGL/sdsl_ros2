from setuptools import find_packages, setup

package_name = 'sdsl_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Bilevich',
    maintainer_email='michaelmoshe@mail.tau.ac.il',
    description='ROS2 Node for the Sparse Distance Sampling Localization (SDSL) algorithm for Online Life-long indoor localization.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
