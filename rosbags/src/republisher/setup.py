from setuptools import setup

package_name = 'republisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cameron',
    maintainer_email='supernovawarriors128@gmail.com',
    description='Examples of minimal publisher/subscriber using rclpy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_fix_republisher = republisher.gps_fix_republisher:main',
            'zed_odom_republisher = republisher.zed_odom_republisher:main',
        ],
    },
)

# Good practice to run rosdep at root of workspace to
# check for missing dependencies before building.
# rosdep install -i --from-path src --rosdistro humble -y
