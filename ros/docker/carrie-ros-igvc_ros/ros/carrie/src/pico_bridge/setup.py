from setuptools import find_packages
from setuptools import setup

package_name = 'pico_bridge'

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
	maintainer='Catherine Van West',
	maintainer_email='catherine.vanwest@cooper.edu',
	description='bridge to the Pico!',
	license='GPLv3 or later',
	tests_require=['pytest'],
	entry_points={
		'console_scripts': [
			'receiver = pico_bridge.command_reception:main',
			'vicon = pico_bridge.vicon_reception:main',
			'bridge = pico_bridge.pico_bridge:main'
		],
	},
)
