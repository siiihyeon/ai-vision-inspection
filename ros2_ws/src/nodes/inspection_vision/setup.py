from setuptools import find_packages, setup

package_name = "inspection_vision"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (
            f"share/{package_name}",
            [
                "package.xml",
                "README.md",
                "코드 읽기 가이드.md",
                "실험_파라미터와_미결정사항.md",
                "MVS_실장비_검증절차.md",
                "검증결과.md",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vision Inspection Team",
    maintainer_email="team@example.com",
    description="Vision capture, durable path queue, worker and ROS adapter.",
    license="Proprietary",
    entry_points={"console_scripts": ["vision_node = inspection_vision.vision_node:main"]},
)
