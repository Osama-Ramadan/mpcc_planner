import os
from glob import glob
from setuptools import setup

package_name = 'igoneo_dynamic_window'
include_name = "dynamic_window"
map_name = "dw_costmap"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,include_name,map_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='mohamed.bakr@still.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dw = igoneo_dynamic_window.igoneo_dynamic_window_node:main'
        ],
    },
)
