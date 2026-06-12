from glob import glob
from setuptools import setup

package_name = 'autonomous_rover'

setup(
    name=package_name,
    version="0.0.1",
    packages=[
        'autonomous_rover',
        'autonomous_rover.nodes',
        'autonomous_rover.nodes.camera',
        'autonomous_rover.nodes.pathfinder',
        'autonomous_rover.nodes.localization',
        'autonomous_rover.nodes.e_comms',
        'autonomous_rover.nodes.master',
        'autonomous_rover.nodes.master.calibration',
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("autonomous_rover/launch/*.py")),
        ("share/" + package_name + "/params", glob("autonomous_rover/params/*.yaml")),
        ("share/" + package_name + "/description/urdf", glob("autonomous_rover/description/urdf/*.xacro")),
        ("share/" + package_name + "/description/meshes", glob("autonomous_rover/description/meshes/*.stl")),
        ("share/" + package_name + "/description/config", glob("autonomous_rover/description/config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer='Purdue EVC',
    maintainer_email='shay.manor@gmail.com',
    license='Apache-2.0',
    description="Package containing all nodes for driving in different states.",
    tests_require=["pytest"],
    entry_points={
        'console_scripts': [
            'camera_node = autonomous_rover.nodes.camera.camera_node:main',
            'pathfinder_node = autonomous_rover.nodes.pathfinder.pathfinder_node:main',
            'localization_node = autonomous_rover.nodes.localization.localization_node:main',
            'e_comms_node = autonomous_rover.nodes.e_comms.e_comms_node:main',
            'master_node = autonomous_rover.nodes.master.master_node:main',
            'calibrate_camera = autonomous_rover.nodes.camera.calibrate_camera:main',
            'compile_depth_qnn = autonomous_rover.nodes.localization.compile_qnn:main',
        ],
    },
)
