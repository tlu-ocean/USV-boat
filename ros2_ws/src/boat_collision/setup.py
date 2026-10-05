from setuptools import find_packages, setup

package_name = 'boat_collision'

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
            'collision_monitor = boat_collision.collision_monitor_node:main',
            'ais_test_target = boat_collision.ais_test_target_node:main',
        ],
    },
)
