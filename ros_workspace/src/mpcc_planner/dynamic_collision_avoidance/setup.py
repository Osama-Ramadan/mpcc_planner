from setuptools import setup

package_name = 'dynamic_collision_avoidance'
dynamic_obstacle_name = 'dynamic_obstacle'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,dynamic_obstacle_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='ossama.ahmed@still.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamic_collision_avoidance = dynamic_collision_avoidance.dynamic_collision_avoidance_node:main'
        ],
    },
)
