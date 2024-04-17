from setuptools import find_packages, setup

package_name = 'controlled_stop_pkg'

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
    maintainer='eeadmin',
    maintainer_email='eeadmin@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # add entry point later 
            'controlled_stop_node = controlled_stop_pkg.controlled_stop_node:main'

        ],
    },
)
