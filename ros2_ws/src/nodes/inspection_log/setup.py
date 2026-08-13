from setuptools import find_packages, setup

package_name = "inspection_log"

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
    description="LogNode package for persistent inspection records.",
    license="Proprietary",
    entry_points={"console_scripts": ["log_node = inspection_log.log_node:main"]},
)

