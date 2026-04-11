from setuptools import find_packages, setup

package_name = 'gimbal_driver'

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
    maintainer='bevyn',
    maintainer_email='evanhennes@gmail.com',
    description='Gimbal control driver',
    license='Tex-Mexticles',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 'GimbalNode = gimbal_driver.gimbal_driver:main'
        ],
    },
)
