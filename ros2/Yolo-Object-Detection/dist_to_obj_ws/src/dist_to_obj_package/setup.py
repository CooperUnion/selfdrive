from setuptools import find_packages, setup

package_name = 'dist_to_obj_package'
submodules = 'dist_to_obj_package/object_detection_submodule'

setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    packages=[package_name,submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'ultralytics'],
    zip_safe=True,

    maintainer='eeadmin',
    maintainer_email='eeadmin@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dist_to_obj_node = dist_to_obj_package.dist_to_obj_node:main'
        ],
    },
)
