from setuptools import find_packages, setup

package_name = 'boat_sensors'

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
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'tfmini_node = boat_sensors.tfmini_node:main',
		'bme280_node = boat_sensors.bme280_node:main',
		'pixhawk_node = boat_sensors.pixhawk_node:main',
		'ais_node = boat_sensors.ais_node:main',
        ],
    },
)
