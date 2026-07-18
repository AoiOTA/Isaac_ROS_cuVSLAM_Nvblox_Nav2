from glob import glob
from setuptools import find_packages, setup


package_name = "jackal_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/urdf", glob("urdf/*.xacro")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
        (f"share/{package_name}/behavior_trees", glob("behavior_trees/*.xml")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="lyb",
    maintainer_email="lyb@example.com",
    description="Jackal visual sensor, TF, and cuVSLAM bringup",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "scan_timestamp_relay = jackal_bringup.scan_timestamp_relay:main",
            "navigation_tf_bridge = jackal_bringup.navigation_tf_bridge:main",
        ]
    },
)
