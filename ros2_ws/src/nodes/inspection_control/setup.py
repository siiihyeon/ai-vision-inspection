from setuptools import find_packages, setup

package_name = "inspection_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vision Inspection Team",
    maintainer_email="team@example.com",
    description="ControlNode package for physical equipment.",
    license="Proprietary",
    entry_points={"console_scripts": ["control_node = inspection_control.control_node:main"]},
)

