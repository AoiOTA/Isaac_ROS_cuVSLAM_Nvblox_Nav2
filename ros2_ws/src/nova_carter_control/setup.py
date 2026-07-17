from glob import glob
from setuptools import find_packages, setup


package_name = "nova_carter_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    test_suite="test",
    zip_safe=True,
    maintainer="lyb",
    maintainer_email="lyb@example.com",
    description="Safe differential-drive command and wheel odometry for Nova Carter",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "command_guard = nova_carter_control.command_guard:main",
            "wheel_odometry = nova_carter_control.wheel_odometry:main",
        ]
    },
)
