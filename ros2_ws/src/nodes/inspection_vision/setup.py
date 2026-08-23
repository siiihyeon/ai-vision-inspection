from setuptools import find_packages, setup

package_name = "inspection_vision"

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
    description="VisionNode package for camera acquisition and inference.",
    license="Proprietary",
    entry_points={"console_scripts": ["vision_node = inspection_vision.vision_node:main"]},
)
