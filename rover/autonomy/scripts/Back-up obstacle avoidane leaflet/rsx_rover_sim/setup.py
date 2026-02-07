from setuptools import find_packages, setup

package_name = 'rsx_rover_sim'

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
    maintainer='raian',
    maintainer_email='raianshayerghafur@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	'csv_world_publisher = rsx_rover_sim.nodes.csv_world_publisher:main',
		'world_state_node = rsx_rover_sim.nodes.world_state_node:main',
		'leaflet_map_ros_ui = rsx_rover_sim.ui.leaflet_map_ros_ui:main',
		'global_planner_node = rsx_rover_sim.planning.global_planner_node:main',
		'rover_sim_node = rsx_rover_sim.sim.rover_sim_node:main',
		'local_detour_planner_node = rsx_rover_sim.planning.local_detour_planner_node:main',
        ],
    },
)
