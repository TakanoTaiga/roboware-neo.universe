from setuptools import find_packages, setup

package_name = 'ros_udp_python_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='taiga',
    maintainer_email='ttttghghnb554z@outlook.jp',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'udp_receive=ros_udp_python_driver.udp_receive:main',
            'udp_send=ros_udp_python_driver.udp_send:main',
        ],
    },
)
