from setuptools import setup

package_name = 'local_path_planner'
map_name = 'mpcc_planner_costmap'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,map_name],
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
            'path_planner = local_path_planner.local_path_planner_node:main'
        ],
    },
)
