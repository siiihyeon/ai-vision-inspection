from setuptools import find_packages, setup

package_name = "node_inspection_simulator"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/inspection_system_sim.launch.py",
                "launch/inspection_system_hardware_result_sim.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vision Inspection Team",
    maintainer_email="team@example.com",
    description="Vision integration simulator for the inspection system.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "vision_simulator_node = node_inspection_simulator.vision_simulator_node:main",
        ]
    },
)
