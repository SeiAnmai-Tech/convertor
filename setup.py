from setuptools import setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'aruco_msgs'],
    zip_safe=True,
    maintainer='sumukh',
    maintainer_email='sumukh.porwal@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plot = py_pubsub.plotting:main',
            'convertor = py_pubsub.convertor:main',
            'odom = py_pubsub.odom_to_pose:main',
            'initialpose = py_pubsub.initial_pose_publisher:main',
            'servo_state = py_pubsub.servo_state_publisher:main',
            'sharp = py_pubsub.sharp:main',
            'sharp_lidar = py_pubsub.sharp_lidar:main',
            'robot_navigator = py_pubsub.robot_navigator:main',
            'gotogoal = py_pubsub.gotogoal:main',
            'goalpub = py_pubsub.goalpub:main',
            'maptoimage = py_pubsub.maptoimage:main',
            'imagecompressor = py_pubsub.imagecompressor:main',
            'rposepub = py_pubsub.rposepub:main',
            'pico_reset = py_pubsub.pico_reset:main',
            'undock = py_pubsub.undock:main',
            'keyboard = py_pubsub.keyboard:main',
        ],
    },
)
