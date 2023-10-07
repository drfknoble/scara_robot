import glob
import os

from setuptools import setup

package_name = 'manipulator_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*')),
        ('share/' + package_name + '/config', glob.glob('config/*')),
        ('share/' + package_name + '/urdf', glob.glob('urdf/*')),
        ('share/' + package_name + '/meshes', glob.glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fknoble',
    maintainer_email='fknoble@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manipulator_control=manipulator_control.manipulator_control:main'
        ],
    },
)
