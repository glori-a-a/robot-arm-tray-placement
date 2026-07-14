from __future__ import annotations

import numpy as np
import mujoco

from simulation.soft_cylinder import SoftCylinder


class ObjectDetector:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, camera_name: str = "overhead_rgb"):
        self.model = model
        self.data = data
        self.camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        self.renderer = mujoco.Renderer(model, height=480, width=640)
        self.cylinder_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "soft_cylinder")
        self.tray_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rectangular_tray")
        self.cylinder_geom_ids = self._collect_body_geom_ids(self.cylinder_body_id)
        self.tray_geom_ids = self._collect_body_geom_ids(self.tray_body_id)
        self.detect_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cylinder_detect")

    def _collect_body_geom_ids(self, body_id: int) -> set[int]:
        geom_ids: set[int] = set()
        for geom_id in range(self.model.ngeom):
            if self.model.geom_bodyid[geom_id] == body_id:
                geom_ids.add(geom_id)
        return geom_ids

    def capture_rgb_image(self) -> np.ndarray:
        self.renderer.disable_depth_rendering()
        self.renderer.disable_segmentation_rendering()
        self.renderer.update_scene(self.data, camera=self.camera_id)
        return self.renderer.render()

    def _segmentation_mask(self, geom_ids: set[int]) -> np.ndarray | None:
        self.renderer.enable_segmentation_rendering()
        self.renderer.update_scene(self.data, camera=self.camera_id)
        segmentation = self.renderer.render()
        self.renderer.disable_segmentation_rendering()

        if segmentation.ndim == 3:
            geom_channel = segmentation[:, :, 0]
        else:
            geom_channel = segmentation

        mask = np.isin(geom_channel, list(geom_ids))
        if not np.any(mask):
            return None
        return mask

    def _pixel_to_world(self, pixel_x: float, pixel_y: float, plane_z: float) -> np.ndarray:
        width = self.renderer.width
        height = self.renderer.height
        cam_id = self.camera_id

        fovy = self.model.cam_fovy[cam_id]
        f = height / (2 * np.tan(np.deg2rad(fovy) / 2))

        cx = width / 2
        cy = height / 2
        x_cam = (pixel_x - cx) / f
        y_cam = (pixel_y - cy) / f
        ray_cam = np.array([x_cam, -y_cam, -1.0])
        ray_cam /= np.linalg.norm(ray_cam)

        cam_pos = self.data.cam_xpos[cam_id].copy()
        cam_mat = self.data.cam_xmat[cam_id].reshape(3, 3).copy()
        ray_world = cam_mat @ ray_cam

        if abs(ray_world[2]) < 1e-6:
            return cam_pos

        scale = (plane_z - cam_pos[2]) / ray_world[2]
        return cam_pos + scale * ray_world

    def _mask_centroid_world(self, mask: np.ndarray, plane_z: float) -> np.ndarray:
        ys, xs = np.where(mask)
        pixel_x = float(np.mean(xs))
        pixel_y = float(np.mean(ys))
        return self._pixel_to_world(pixel_x, pixel_y, plane_z=plane_z)

    def _detect_cylinder(self, table_z: float) -> np.ndarray | None:
        # Prefer the localisation beacon (compact sphere at body XY).
        if self.detect_geom_id >= 0:
            mask = self._segmentation_mask({self.detect_geom_id})
            if mask is not None and int(np.count_nonzero(mask)) >= 8:
                plane_z = float(self.data.geom_xpos[self.detect_geom_id][2])
                hit = self._mask_centroid_world(mask, plane_z=plane_z)
                hit[2] = SoftCylinder.center_z_on_surface(table_z)
                return hit

        # Fallback: full body silhouette projected to body-centre height.
        mask = self._segmentation_mask(self.cylinder_geom_ids)
        if mask is None:
            return None
        plane_z = SoftCylinder.center_z_on_surface(table_z)
        return self._mask_centroid_world(mask, plane_z=plane_z)

    def _detect_tray(self, table_z: float) -> np.ndarray | None:
        mask = self._segmentation_mask(self.tray_geom_ids)
        if mask is None:
            return None
        return self._mask_centroid_world(mask, plane_z=table_z)

    def detect(self, table_z: float = 0.40) -> dict:
        # detect objects
        rgb_image = self.capture_rgb_image()
        cylinder_position = self._detect_cylinder(table_z=table_z)
        tray_position = self._detect_tray(table_z=table_z)

        return {
            "image": rgb_image,
            "cylinder_mask": None,
            "tray_mask": None,
            "cylinder_position": cylinder_position,
            "tray_position": tray_position,
        }

    def close(self) -> None:
        self.renderer.close()
