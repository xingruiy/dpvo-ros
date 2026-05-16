from setuptools import find_packages, setup
import os

package_name = 'dpvo_ros'

launch_files = []
for file in os.listdir('launch'):
    if file.endswith('.launch.py'):
        launch_files.append('launch/' + file)

config_files = []
for file in os.listdir('config'):
    if file.endswith('.yaml'):
        config_files.append('config/' + file)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', launch_files),
        ('share/' + package_name + '/config', config_files),
        ('share/' + package_name + '/network', ['network/dpvo.pth']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='64749983+xingruiy@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dpvo_node = dpvo_ros.dpvo_node:main'
        ],
    },
)
