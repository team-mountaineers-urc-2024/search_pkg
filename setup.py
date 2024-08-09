from setuptools import find_packages, setup

package_name = 'search_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='z',
    maintainer_email='zspiggle@outlook.com',
    description='A node to handle the management of a Probability Distribution Function to make suggestions of the best locations to search.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'search_node = search_pkg.ros2_search_node:main',
            'search_test = search_pkg.ros2_testing_node:main',
        ],
    },
)
