from setuptools import find_packages, setup


package_name = "nova_carter_experiments"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
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
    description="Automated Nova Carter motion, sensing, SLAM, and nvblox experiments",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "motion_test_runner = nova_carter_experiments.motion_test_runner:main",
            "sensor_test_runner = nova_carter_experiments.sensor_test_runner:main",
            "sensor_payload_probe = nova_carter_experiments.sensor_payload_probe:main",
            "visual_slam_test_runner = nova_carter_experiments.visual_slam_test_runner:main",
            "nvblox_map_saver = nova_carter_experiments.nvblox_map_saver:main",
            "nvblox_test_runner = nova_carter_experiments.nvblox_test_runner:main",
            "occupancy_saver = nova_carter_experiments.occupancy_saver:main",
            "vgl_pose_relay = nova_carter_experiments.vgl_pose_relay:main",
            "vgl_test_runner = nova_carter_experiments.vgl_test_runner:main",
            "localization_bootstrap = nova_carter_experiments.localization_bootstrap:main",
            "navigation_test_runner = nova_carter_experiments.navigation_test_runner:main",
        ]
    },
)
