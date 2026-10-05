import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'boat_bringup'


setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [
                'resource/' + package_name
            ]
        ),

        (
            'share/' + package_name,
            [
                'package.xml'
            ]
        ),

        (
            os.path.join(
                'share',
                package_name,
                'launch',
            ),
            glob('launch/*.launch.py')
        ),

        (
            os.path.join(
                'share',
                package_name,
                'config',
            ),
            glob('config/*.yaml')
        ),
    ],

    install_requires=[
        'setuptools'
    ],

    zip_safe=True,

    maintainer='tianyi',

    maintainer_email='TODO',

    description=(
        'Bringup package for the USV'
    ),

    license='Apache-2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'boat_health = boat_bringup.boat_health_node:main',
        ],
    },
)
