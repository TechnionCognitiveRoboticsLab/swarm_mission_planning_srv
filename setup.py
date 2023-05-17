from setuptools import setup

package_name = 'mission_planner_srv'

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
    maintainer='karpase',
    maintainer_email='karpase@gmail.com',
    description='Mission Planning Service',
    license='FML',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'service = mission_planner_srv.lawnmower:main',
        'client = mission_planner_srv.get_mission_plan:main',
        'client_polygons = mission_planner_srv.get_mission_plan_polygons:main',
        'service_polygons = mission_planner_srv.server_polygons:main',
    ],
},
)
