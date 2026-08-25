from setuptools import find_packages, setup

package_name = "inspection_master"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (
            f"share/{package_name}",
            ["package.xml", "README.md", "마스터노드_읽기가이드.md"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vision Inspection Team",
    maintainer_email="team@example.com",
    description="MasterNode package for system orchestration and product tracking.",
    license="Proprietary",
    entry_points={"console_scripts": ["master_node = inspection_master.master_node:main"]},
)
