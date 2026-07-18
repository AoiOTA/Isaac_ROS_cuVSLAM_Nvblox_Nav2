from jackal_experiments.nav2_lifecycle_guard import Nav2LifecycleGuard


def test_all_active_requires_every_managed_node() -> None:
    assert Nav2LifecycleGuard.all_active({"planner": "active", "controller": "active"})
    assert not Nav2LifecycleGuard.all_active(
        {"planner": "active", "controller": "inactive"}
    )
    assert not Nav2LifecycleGuard.all_active({})
