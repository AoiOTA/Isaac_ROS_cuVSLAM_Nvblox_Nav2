"""Smooth GUI-only third-person camera for the runtime-composed Jackal."""

from __future__ import annotations

import math

from pxr import Gf, Sdf, Usd, UsdGeom


CAMERA_PATH = "/World/FollowCameraRig"


class FollowCamera:
    def __init__(
        self,
        stage: Usd.Stage,
        target_path: str,
        *,
        distance_m: float = 3.0,
        height_m: float = 1.8,
        target_height_m: float = 0.4,
        smoothing_time_s: float = 0.25,
    ) -> None:
        self.stage = stage
        self.target = stage.GetPrimAtPath(target_path)
        if not self.target.IsValid():
            raise RuntimeError(f"follow-camera target is invalid: {target_path}")
        self.distance = distance_m
        self.height = height_m
        self.target_height = target_height_m
        self.smoothing_time = smoothing_time_s
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            camera = UsdGeom.Camera.Define(stage, CAMERA_PATH)
            camera.CreateFocalLengthAttr(20.0)
            camera.CreateHorizontalApertureAttr(20.955)
            camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 10000.0))
            self.transform = camera.AddTransformOp()
        self.eye: Gf.Vec3d | None = None
        self.look_at: Gf.Vec3d | None = None
        self.update(1.0)

    def desired_pose(self) -> tuple[Gf.Vec3d, Gf.Vec3d]:
        matrix = UsdGeom.Xformable(self.target).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        base = matrix.ExtractTranslation()
        forward = matrix.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0)).GetNormalized()
        eye = base - forward * self.distance + Gf.Vec3d(0.0, 0.0, self.height)
        target = base + Gf.Vec3d(0.0, 0.0, self.target_height)
        return eye, target

    @staticmethod
    def blend(current: Gf.Vec3d, desired: Gf.Vec3d, alpha: float) -> Gf.Vec3d:
        return current * (1.0 - alpha) + desired * alpha

    def update(self, dt: float) -> None:
        desired_eye, desired_target = self.desired_pose()
        alpha = 1.0 if self.eye is None else 1.0 - math.exp(-max(dt, 0.0) / self.smoothing_time)
        self.eye = desired_eye if self.eye is None else self.blend(self.eye, desired_eye, alpha)
        self.look_at = (
            desired_target
            if self.look_at is None
            else self.blend(self.look_at, desired_target, alpha)
        )
        camera_to_world = Gf.Matrix4d().SetLookAt(
            self.eye, self.look_at, Gf.Vec3d(0.0, 0.0, 1.0)
        ).GetInverse()
        self.transform.Set(camera_to_world)

    def state(self) -> dict[str, list[float]]:
        """Return the currently authored camera and focus positions for reporting."""

        if self.eye is None or self.look_at is None:
            raise RuntimeError("follow camera was queried before its first update")
        return {
            "eye_m": [float(value) for value in self.eye],
            "look_at_m": [float(value) for value in self.look_at],
        }


def activate_viewport_camera() -> None:
    from omni.kit.viewport.utility import get_active_viewport

    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("Isaac Sim GUI has no active viewport")
    # Isaac Sim 6 uses the ViewportAPI camera_path property.  set_active_camera
    # belongs to an older viewport wrapper and can leave the visible viewport
    # on its perspective camera after the timeline starts.
    viewport.camera_path = Sdf.Path(CAMERA_PATH)
    if str(viewport.camera_path) != CAMERA_PATH:
        raise RuntimeError(
            f"failed to bind GUI viewport to follow camera: {viewport.camera_path}"
        )
