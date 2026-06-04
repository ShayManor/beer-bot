from glob import glob
from setuptools import setup

package_name = 'beer_bot'

setup(
    name=package_name,
    version="0.0.1",
    packages=[
        'beer_bot',
        'beer_bot.nodes',
        'beer_bot.nodes.camera',
        'beer_bot.nodes.pathfinder',
        'beer_bot.nodes.localization',
        'beer_bot.nodes.e_comms',
        'beer_bot.nodes.master',
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("beer_bot/launch/*.py")),
        ("share/" + package_name + "/params", glob("beer_bot/params/*.yaml")),
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
            'camera_node = beer_bot.nodes.camera.camera_node:main',
            'pathfinder_node = beer_bot.nodes.pathfinder.pathfinder_node:main',
            'localization_node = beer_bot.nodes.localization.localization_node:main',
            'e_comms_node = beer_bot.nodes.e_comms.e_comms_node:main',
            'master_node = beer_bot.nodes.master.master_node:main',
        ],
    },
)
