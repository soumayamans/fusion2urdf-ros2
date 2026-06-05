from setuptools import setup
import os
from glob import glob

package_name = 'fusion2urdf_ros2'


def get_data_files(source_dirs):
    result = []
    for source_dir in source_dirs:
        for dirpath, _, filenames in os.walk(source_dir):
            if filenames:
                dest = os.path.join('share', package_name, dirpath)
                sources = [os.path.join(dirpath, f) for f in filenames]
                result.append((dest, sources))
    return result


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ] + get_data_files(['urdf', 'meshes']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='The ' + package_name + ' package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
