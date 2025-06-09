from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'mouse_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools', 'pynput'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='yatracker.ol00lo@yandex.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
    'console_scripts': [
        'mouse_sensor = mouse_control.mouse_sensor:main',
    ],

},

)
