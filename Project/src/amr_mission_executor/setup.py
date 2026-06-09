import os
from glob import glob
from setuptools import find_packages, setup

package_name = "amr_mission_executor"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"),
            glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="suhaib",
    maintainer_email="suhaib@example.com",
    description="Mission executor — bridges ExecuteMission to Nav2 NavigateToPose.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_executor = amr_mission_executor.mission_node:main",
        ],
    },
)
