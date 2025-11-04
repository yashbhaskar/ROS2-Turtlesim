from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'shapes_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share",package_name,"launch"),glob("launch/*")),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yash',
    maintainer_email='yash@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'triangle = shapes_pkg.triangle:main',
        	'circle = shapes_pkg.circle:main',
        	'square = shapes_pkg.square:main',
        	'star = shapes_pkg.star:main',
        	'star12 = shapes_pkg.12_star:main',
        	'hexagon = shapes_pkg.hexagon:main',
        	'octagon = shapes_pkg.octagon:main',
        	'main = shapes_pkg.main:main',
        	'clear = shapes_pkg.clear:main',
            'point = shapes_pkg.point:main',
            'teleport = shapes_pkg.teleport:main',
            'spawn = shapes_pkg.spawn:main',
            'kill = shapes_pkg.kill:main',
            'random = shapes_pkg.random:main',
            'turtle_spawn = shapes_pkg.turtlesim_spawn:main'

        ],
    },
)
