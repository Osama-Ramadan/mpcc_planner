import os
from glob import glob
from setuptools import setup

package_name = 'mpc_igoneo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ossama Ahmed',
    maintainer_email='ossama.ahmed@still.de',
    description='Package to control the IgoNeo on ISAACSIM',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planner = mpc_igoneo.path_planner:main',
            'mpc = mpc_igoneo.mpcc_controller:main'
        ],
    },
    
)

