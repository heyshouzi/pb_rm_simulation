from setuptools import find_packages, setup


package_name = "rm_rl_control"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/mode_a_rl.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rm_rl_control",
    maintainer_email="noreply@example.com",
    description="RL navigation controller (Mode A) for pb_rm_simulation (ROS2 Humble).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rl_controller = rm_rl_control.rl_controller_node:main",
        ],
    },
)

