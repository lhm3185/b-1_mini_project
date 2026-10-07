from glob import glob

from setuptools import find_packages, setup

package_name = 'navi_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='doyoon-kim',
    maintainer_email='tnduaehowl37@gmail.com',
    description='미니 프로젝트 AMR 주행: 검출 신호 수신, 지정 좌표 이동, scan_motion, RC카 추종',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'follow_car = navi_pkg.follow_car:main',
        ],
    },
)
