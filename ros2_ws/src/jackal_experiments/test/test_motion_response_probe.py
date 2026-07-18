import pytest

from jackal_experiments.motion_response_probe import (
    PoseSample,
    angle_delta,
    evaluate_arc,
    evaluate_spin,
)


def test_angle_delta_wraps_at_pi() -> None:
    assert angle_delta(-3.1, 3.1) > 0.0


def test_reference_quality_spin_passes() -> None:
    samples = [
        PoseSample(index * 0.1, 0.0, 0.0, index * 0.08, 0.8)
        for index in range(33)
    ]
    result = evaluate_spin(
        samples,
        command_angular=0.8,
        duration_s=3.2,
        maximum_observed_command=0.8,
    )
    assert result["status"] == "passed"


def test_understeering_spin_fails() -> None:
    samples = [
        PoseSample(index * 0.1, 0.0, 0.0, index * 0.02, 0.2)
        for index in range(33)
    ]
    result = evaluate_spin(
        samples,
        command_angular=0.8,
        duration_s=3.2,
        maximum_observed_command=0.8,
    )
    assert result["status"] == "failed"
    assert result["checks"]["yaw_tracking"] is False


def test_reference_quality_arc_passes() -> None:
    samples = [
        PoseSample(
            index * 0.05,
            index * 0.016,
            0.0,
            index * 0.04,
            0.8,
            0.32,
        )
        for index in range(81)
    ]
    result = evaluate_arc(
        samples,
        command_linear=0.32,
        command_angular=0.8,
        duration_s=4.0,
        maximum_observed_linear=0.32,
        maximum_observed_angular=0.8,
    )
    assert result["status"] == "passed"
    assert result["actual_radius_m"] == pytest.approx(0.4)


def test_understeering_arc_fails_curvature_check() -> None:
    samples = [
        PoseSample(
            index * 0.05,
            index * 0.016,
            0.0,
            index * 0.01,
            0.2,
            0.32,
        )
        for index in range(81)
    ]
    result = evaluate_arc(
        samples,
        command_linear=0.32,
        command_angular=0.8,
        duration_s=4.0,
        maximum_observed_linear=0.32,
        maximum_observed_angular=0.8,
    )
    assert result["status"] == "failed"
    assert result["checks"]["curvature_tracking"] is False
