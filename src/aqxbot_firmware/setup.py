from setuptools import find_packages, setup

package_name = 'aqxbot_firmware'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fahad',
    maintainer_email='gik.fahad@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rplidar_receiver = aqxbot_firmware.rplidar_receiver:main',
            'tf_test = aqxbot_firmware.tf_test:main',
            'imu_receiver = aqxbot_firmware.imu_receiver:main',
        ],
    },
)
