from setuptools import find_packages, setup

package_name = "inspection_common"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vision Inspection Team",
    maintainer_email="team@example.com",
    description="Minimal shared Python utilities for inspection nodes.",
    license="Proprietary",
)
