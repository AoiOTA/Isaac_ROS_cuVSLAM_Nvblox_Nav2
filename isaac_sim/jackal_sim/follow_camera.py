"""Robot-root-attached third-person camera for the runtime-composed Jackal."""

from __future__ import annotations

from pxr import Gf, Sdf, Usd, UsdGeom


CAMERA_NAME = "FollowCameraRig"
CAMERA_PATH = f"/World/{CAMERA_NAME}"
REFERENCE_DISTANCE_M = 3.2
REFERENCE_HEIGHT_M = 2.2
REFERENCE_LOOK_AHEAD_M = 1.0
REFERENCE_LOOK_AT_HEIGHT_M = 0.25
REFERENCE_FOCAL_LENGTH_MM = 16.0


class FollowCamera:
    def __init__(
        self,
        stage: Usd.Stage,
        target_path: str,
        *,
        distance_m: float = REFERENCE_DISTANCE_M,
        height_m: float = REFERENCE_HEIGHT_M,
        look_ahead_m: float = REFERENCE_LOOK_AHEAD_M,
        target_height_m: float = REFERENCE_LOOK_AT_HEIGHT_M,
        focal_length_mm: float = REFERENCE_FOCAL_LENGTH_MM,
        smoothing_time_s: float = 0.25,
    ) -> None:
        self.stage = stage
        self.target = stage.GetPrimAtPath(target_path)
        if not target_path.startswith("/"):
            raise RuntimeError(
                f"follow-camera target must be absolute USD path: {target_path}"
            )
        if not self.target.IsValid():
            raise RuntimeError(f"follow-camera target is invalid: {target_path}")
        self.distance = distance_m
        self.height = height_m
        self.look_ahead = look_ahead_m
        self.target_height = target_height_m
        self.focal_length = focal_length_mm
        self.smoothing_time = smoothing_time_s
        self.camera_path = f"{target_path.rstrip('/')}" f"/{CAMERA_NAME}"
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            camera = UsdGeom.Camera.Define(stage, self.camera_path)
            camera.CreateFocalLengthAttr(self.focal_length)
            camera.CreateHorizontalApertureAttr(20.955)
            camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 10000.0))
            self.transform = camera.AddTransformOp()
        self.camera = camera
        self._apply_local_pose()

    def desired_pose(self) -> tuple[Gf.Vec3d, Gf.Vec3d]:
        matrix = UsdGeom.Xformable(self.target).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        base = matrix.ExtractTranslation()
        forward = matrix.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0)).GetNormalized()
        eye = base - forward * self.distance + Gf.Vec3d(0.0, 0.0, self.height)
        target = (
            base
            + forward * self.look_ahead
            + Gf.Vec3d(0.0, 0.0, self.target_height)
        )
        return eye, target

    @staticmethod
    def blend(current: Gf.Vec3d, desired: Gf.Vec3d, alpha: float) -> Gf.Vec3d:
        return current * (1.0 - alpha) + desired * alpha

    def _local_pose_matrix(self) -> Gf.Matrix4d:
        # `eye`/`aim` are robot-local; parenting to ``target_path`` keeps
        # the camera tracking translation/rotation automatically, without any
        # per-frame matrix copy.
        return Gf.Matrix4d(1.0).SetLookAt(
            Gf.Vec3d(-self.distance, 0.0, self.height),
            Gf.Vec3d(self.look_ahead, 0.0, self.target_height),
            Gf.Vec3d(0.0, 0.0, 1.0),
        ).GetInverse().GetOrthonormalized()

    def _apply_local_pose(self) -> None:
        self.transform.Set(self._local_pose_matrix())

    def _apply_camera_profile(self) -> None:
        self.camera.CreateFocalLengthAttr().Set(self.focal_length)

    def update(self, dt: float) -> None:
        del dt

    def reconfigure(
        self,
        *,
        distance_m: float | None = None,
        height_m: float | None = None,
        look_ahead_m: float | None = None,
        target_height_m: float | None = None,
        focal_length_mm: float | None = None,
        smoothing_time_s: float | None = None,
    ) -> None:
        if distance_m is not None:
            if distance_m <= 0.0:
                raise ValueError("distance must be positive")
            self.distance = distance_m
        if height_m is not None:
            if height_m < 0.0:
                raise ValueError("height must be non-negative")
            self.height = height_m
        if look_ahead_m is not None:
            if look_ahead_m < 0.0:
                raise ValueError("look-ahead must be non-negative")
            self.look_ahead = look_ahead_m
        if target_height_m is not None:
            if target_height_m < 0.0:
                raise ValueError("look-at height must be non-negative")
            self.target_height = target_height_m
        if focal_length_mm is not None:
            if focal_length_mm <= 0.0:
                raise ValueError("focal length must be positive")
            self.focal_length = focal_length_mm
            self._apply_camera_profile()
        if smoothing_time_s is not None:
            if smoothing_time_s <= 0.0:
                raise ValueError("smoothing time must be positive")
            self.smoothing_time = smoothing_time_s
        self._apply_local_pose()

    def state(self) -> dict[str, list[float]]:
        """Return the currently authored camera and focus positions for reporting."""

        eye, target = self.desired_pose()
        return {
            "eye_m": [float(value) for value in eye],
            "look_at_m": [float(value) for value in target],
        }

    def profile(self) -> dict[str, float]:
        """Return the reference-aligned optical geometry for reports and checks."""

        return {
            "distance_m": self.distance,
            "height_m": self.height,
            "look_ahead_m": self.look_ahead,
            "look_at_height_m": self.target_height,
            "focal_length_mm": self.focal_length,
        }

    def bind_viewport(self) -> bool:
        """Bind the active viewport to this camera if it exists."""

        from omni.kit.viewport.utility import get_active_viewport

        viewport = get_active_viewport()
        if viewport is None:
            return False
        # Keep viewport interactions (zoom/drag/orbit) enabled so operators can
        # tune this camera at runtime without being continuously overridden.
        viewport.updates_enabled = True
        # Isaac Sim 6 uses the ViewportAPI camera_path property.  set_active_camera
        # belongs to an older viewport wrapper and can leave the visible viewport
        # on its perspective camera after the timeline starts.
        viewport.camera_path = Sdf.Path(self.camera_path)
        return str(viewport.camera_path) == self.camera_path



def activate_viewport_camera(camera_path: str = CAMERA_PATH) -> bool:
    from omni.kit.viewport.utility import get_active_viewport

    viewport = get_active_viewport()
    if viewport is None:
        return False
    viewport.updates_enabled = True
    # Isaac Sim 6 uses the ViewportAPI camera_path property.  set_active_camera
    # belongs to an older viewport wrapper and can leave the visible viewport
    # on its perspective camera after the timeline starts.
    viewport.camera_path = Sdf.Path(camera_path)
    return str(viewport.camera_path) == camera_path
