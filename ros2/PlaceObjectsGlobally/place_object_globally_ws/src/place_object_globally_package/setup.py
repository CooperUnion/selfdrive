from setuptools import find_packages, setup

package_name = 'place_object_globally_package'

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
    maintainer_email='vaibhav.hariani@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'place_object_globally_node = place_object_globally_package.place_object_globally_node:main'
        ],
    },
)
