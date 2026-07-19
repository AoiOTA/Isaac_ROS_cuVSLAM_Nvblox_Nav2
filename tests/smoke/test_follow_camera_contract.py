import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
CAMERA_SOURCE = ROOT / "isaac_sim/jackal_sim/follow_camera.py"


def test_follow_camera_matches_reference_branch_geometry() -> None:
    source = CAMERA_SOURCE.read_text(encoding="utf-8")
    tree = ast.parse(source)
    constants = {}
    for node in tree.body:
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        target = node.targets[0]
        if isinstance(target, ast.Name) and target.id.startswith("REFERENCE_"):
            constants[target.id] = ast.literal_eval(node.value)

    assert constants == {
        "REFERENCE_DISTANCE_M": 3.2,
        "REFERENCE_HEIGHT_M": 2.2,
        "REFERENCE_LOOK_AHEAD_M": 1.0,
        "REFERENCE_LOOK_AT_HEIGHT_M": 0.25,
        "REFERENCE_FOCAL_LENGTH_MM": 16.0,
    }
    assert "forward * self.look_ahead" in source


def test_simulator_report_uses_the_live_camera_profile() -> None:
    source = (ROOT / "isaac_sim/navigation_sim.py").read_text(encoding="utf-8")
    assert "**follow_camera.profile()" in source
