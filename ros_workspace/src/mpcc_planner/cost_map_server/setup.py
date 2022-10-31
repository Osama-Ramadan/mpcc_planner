import os
from glob import glob
from setuptools import setup

package_name = 'cost_map_server'
include_name = "include_cost"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,include_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
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
            "cost_map_server = cost_map_server.cost_map_server_node:main"
        ],
    },
)
