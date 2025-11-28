from setuptools import find_packages, setup

package_name = 'auto_explore'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='second-code',
    maintainer_email='contact.ashi.gupta@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'map_listener = auto_explore.map_listener:main',
            'waypoint_generator = auto_explore.waypoint_generator:main',
            'waypoint_executor = auto_explore.waypoint_executor:main',
        ],
    },

)
