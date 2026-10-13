import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_nav_tests'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Συμπερίληψη του test_thresholds.json στο share directory
        (os.path.join('share', package_name), glob('my_nav_tests/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Automated Nav2 Metrics & Regression Testing Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'metrics_collector = my_nav_tests.metrics_collector:main',
            'regression_checker = my_nav_tests.regression_checker:run_regression_check',
        ],
    },
)
