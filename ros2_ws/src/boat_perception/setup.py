from setuptools import find_packages, setup

package_name = 'boat_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tianyi Lu',
    maintainer_email='luty8@tamu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'lidar_sector_node = boat_perception.lidar_sector_node:main',
		'lidar_obstacle_node = boat_perception.lidar_obstacle_node:main',
        ],
    },
)
