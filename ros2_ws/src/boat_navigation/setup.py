from setuptools import find_packages, setup

package_name = 'boat_navigation'

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
            'avoidance_decision_node = boat_navigation.avoidance_decision_node:main',
            'guided_target_node = boat_navigation.guided_target_node:main',
            'local_planner_node = boat_navigation.local_planner_node:main',
        ],
    },
)
