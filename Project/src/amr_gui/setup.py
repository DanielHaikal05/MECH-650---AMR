import os
from glob import glob
from setuptools import find_packages, setup

package_name = "amr_gui"

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="boulos",
    maintainer_email="boulos@example.com",
    description="Mission Console GUI",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_console = amr_gui.mission_console:main",
            "mock_mission_server = amr_gui.mock_mission_server:main",
        ],
    },
)
