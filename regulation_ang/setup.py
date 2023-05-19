from setuptools import setup

package_name = 'regulation_ang'

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
    description='This package creates a graph for angular step velocity.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'regulation_ang = regulation_ang.regulation_ang:main'
        ],
    },
)
