from setuptools import find_packages, setup

package_name = 'sample_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'geometry_msgs',
        'control_msgs',
        'std_srvs',
        'scipy',
        'transforms3d',
    ],
    zip_safe=True,
    maintainer='pc',
    maintainer_email='pc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'keyboard_move_sample = sample_package.keyboard_move_sample:main',
            'test = sample_package.test:main',
            'roboter_template = sample_package.roboter_template:main',
        ],
    },
)
