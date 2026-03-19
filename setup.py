from setuptools import find_packages, setup

package_name = "drone_slam"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", [
            "config/scene.yaml",
            "config/flight_path.yaml",
            "config/cuVSLAM.yaml",
        ]),
        ("share/" + package_name + "/launch", [
            "launch/slam.launch.py",
            "launch/sim.launch.py",
            "launch/full.launch.py",
        ]),
        ("share/" + package_name + "/rviz", [
            "rviz/slam_view.rviz",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "map_saver = drone_slam.map_saver:main",
        ],
    },
)
