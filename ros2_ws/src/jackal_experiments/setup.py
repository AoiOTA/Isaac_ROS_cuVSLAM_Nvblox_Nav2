from setuptools import find_packages, setup


package_name = "jackal_experiments"

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
    description="Automated Jackal motion, sensing, SLAM, and nvblox experiments",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "motion_test_runner = jackal_experiments.motion_test_runner:main",
            "sensor_test_runner = jackal_experiments.sensor_test_runner:main",
            "sensor_payload_probe = jackal_experiments.sensor_payload_probe:main",
            "visual_slam_test_runner = jackal_experiments.visual_slam_test_runner:main",
            "visual_map_saver = jackal_experiments.visual_map_saver:main",
            "nvblox_map_saver = jackal_experiments.nvblox_map_saver:main",
            "nvblox_test_runner = jackal_experiments.nvblox_test_runner:main",
            "occupancy_saver = jackal_experiments.occupancy_saver:main",
            "vgl_pose_relay = jackal_experiments.vgl_pose_relay:main",
            "vgl_test_runner = jackal_experiments.vgl_test_runner:main",
            "localization_bootstrap = jackal_experiments.localization_bootstrap:main",
            "navigation_test_runner = jackal_experiments.navigation_test_runner:main",
            "performance_workload_driver = jackal_experiments.performance_workload_driver:main",
            "mapping_coverage_driver = jackal_experiments.mapping_coverage_driver:main",
            "motion_response_probe = jackal_experiments.motion_response_probe:main",
            "localization_recovery_manager = jackal_experiments.localization_recovery_manager:main",
            "resilient_navigation = jackal_experiments.resilient_navigation:main",
            "manual_goal_bridge = jackal_experiments.manual_goal_bridge:main",
            "nav2_lifecycle_guard = jackal_experiments.nav2_lifecycle_guard:main",
        ]
    },
)
