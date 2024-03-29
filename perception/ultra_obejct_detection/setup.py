from setuptools import find_packages, setup

package_name = 'ultra_obejct_detection'

setup(
    name=package_name,
    version='0.0.0',
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='taiga',
    maintainer_email='ttttghghnb554z@outlook.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'rt_detr = ultra_obejct_detection.rt_detr:main'
        ],
    },
)
