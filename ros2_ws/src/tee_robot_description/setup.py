from setuptools import setup
from glob import glob
import os

package_name = 'tee_robot_description'

def g(p):
    return glob(p) if glob(p) else []

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'urdf'), g('urdf/*')),
    (os.path.join('share', package_name, 'launch'), g('launch/*')),
]

if os.path.isdir('config'):
    data_files.append((os.path.join('share', package_name, 'config'), g('config/*')))
if os.path.isdir('meshes'):
    data_files.append((os.path.join('share', package_name, 'meshes'), g('meshes/*')))

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nikitha',
    maintainer_email='nikitha@example.com',
    description='TEE robot URDF/xacro description package',
    license='MIT',
)
