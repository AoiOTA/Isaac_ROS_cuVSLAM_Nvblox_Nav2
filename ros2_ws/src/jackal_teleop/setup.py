from setuptools import find_packages, setup

package_name = "jackal_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    test_suite="test",
    zip_safe=True,
    maintainer="lyb",
    maintainer_email="lyb@example.com",
    description="Deadman-protected Jackal keyboard teleop",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "keyboard_teleop = jackal_teleop.keyboard_teleop:main",
        ]
    },
)
