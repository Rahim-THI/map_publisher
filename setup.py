from setuptools import setup

package_name = 'map_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'lanelet_visualizer = map_publisher.lanelet_visualizer:main',
            'object_publisher = map_publisher.object_publisher:main',
        ],
    },
)
