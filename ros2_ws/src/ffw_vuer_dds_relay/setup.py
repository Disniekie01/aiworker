from glob import glob
import os

from setuptools import find_packages, setup

package_name = "ffw_vuer_dds_relay"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "assets"), glob("assets/*.urdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robotis_vr_isaac",
    maintainer_email="user@example.com",
    description="Vuer ROS2 to DDS relay for FFW sim teleop",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "vuer_dds_relay_node = ffw_vuer_dds_relay.vuer_dds_relay_node:main",
        ],
    },
)
