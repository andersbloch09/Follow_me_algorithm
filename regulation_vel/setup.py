from setuptools import setup

package_name = 'regulation_vel'

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
    maintainer='Anders Bloch Lauridsen',
    maintainer_email='andersbloch09@gmail.com',
    description='Small package to observe regulation of turtlebot3 linear velocity.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['reg_vel = regulation_vel.reg_vel:main',
        ],
    },
)
