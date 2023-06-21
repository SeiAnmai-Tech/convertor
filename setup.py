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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sumukh',
    maintainer_email='sumukh.porwal@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'publisher = py_pubsub.publisher_member_function:main',
        	'listener = py_pubsub.subscriber_member_function:main',
            'plot = py_pubsub.plotting:main',
            'convertor = py_pubsub.convertor:main',
            'odom = py_pubsub.odom_to_pose:main',
        ],
    },
)