from setuptools import find_packages, setup
import os
from glob import glob

package_name = "crisp_controllers_robot_demos"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), [f for f in glob("launch/*") if os.path.isfile(f)]),
        *[
            (os.path.join("share", package_name, os.path.dirname(f)), [f])
            for f in glob("config/**", recursive=True)
            if os.path.isfile(f)
        ],
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="42489409+danielsanjosepro@users.noreply.github.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "target_publisher = crisp_controllers_robot_demos.target_publisher:main",
            "crisp_py_franka_hand_adapter = crisp_controllers_robot_demos.crisp_py_franka_hand_adapter:main",
            "external_effort_node = crisp_controllers_robot_demos.external_effort_node:main",
            "calibrate_external_effort = crisp_controllers_robot_demos.calibrate_external_effort:main",
            "identify_payload = crisp_controllers_robot_demos.identify_payload:main",
            "velocity_sweep = crisp_controllers_robot_demos.velocity_sweep:main",
        ],
    },
)
