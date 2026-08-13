"""설치된 ROS 2 패키지의 실제 버전을 읽습니다."""

from pathlib import Path
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory


def read_installed_package_version(package_name: str) -> str:
    """ament index가 가리키는 package.xml의 version을 반환합니다."""

    package_xml = Path(get_package_share_directory(package_name)) / "package.xml"
    version = ElementTree.parse(package_xml).getroot().findtext("version")
    if not version:
        raise RuntimeError(f"package version is missing: {package_name}")
    return version
