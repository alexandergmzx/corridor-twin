import os
from glob import glob

from setuptools import setup

package_name = "corridor_scene"

setup(
    name=package_name,
    version="0.1.0",
    packages=["scene"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alexander Gomez",
    maintainer_email="alexandergmzx@gmail.com",
    description="GPU-independent OpenUSD scene authoring for corridor-twin.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "corridor-build = scene.build:main",
            "corridor-occlusion = scene.occlusion:main",
        ]
    },
)
