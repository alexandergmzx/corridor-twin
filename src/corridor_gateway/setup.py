import os
from glob import glob

from setuptools import find_packages, setup

package_name = "corridor_gateway"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alexander Gomez",
    maintainer_email="alexandergmzx@gmail.com",
    description="The one sanctioned one-way crossing between the robot and police ROS domains.",
    license="Apache-2.0",
    tests_require=["pytest"],
)
