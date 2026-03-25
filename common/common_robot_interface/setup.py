from setuptools import find_packages, setup

package_name = 'common_robot_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='csi',
    maintainer_email='seonilchoi98@gmail.com',
    description='Shared Python robot abstractions (Action)',
    license='MIT',
)
