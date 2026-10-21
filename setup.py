from setuptools import find_packages, setup
import os
from glob import glob

setup(
    name='my_nav_tests',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/my_nav_tests']),
        ('share/my_nav_tests', ['package.xml']),
        (os.path.join('share', 'my_nav_tests', 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', 'my_nav_tests', 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LefterisPet',
    maintainer_email='user@todo.todo',
    description='Nav2 regression testing package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Πρόσθεσε εδώ τα executables σου αν έχεις (π.χ. 'test_node = my_nav_tests.test_node:main')
        ],
    },
)
