from setuptools import find_packages, setup

package_name = 'ultra_object_detection'

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
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rt_detr = ultra_object_detection.rt_detr:main',
            'yolo = ultra_object_detection.yolo:main',
        ],
    },
)
