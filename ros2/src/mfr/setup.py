from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mfr'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='melih',
    maintainer_email='melih.akay@metu.edu.tr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "joy_controller=mfr_control.joy_controller:main"
        ],
    },
)
