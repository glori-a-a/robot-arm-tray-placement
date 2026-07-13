"""RGB camera sensor for detecting the cylinder and tray."""

from __future__ import annotations

import numpy as np
import cv2
import mujoco


class ObjectDetector:
    """Finds the soft cylinder and rectangular tray from an overhead camera."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, camera_name: str = "overhead_rgb"):
        self.model = model
        self.data = data
        self.camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        self.renderer = mujoco.Renderer(model, height=480, width=640)
        self.cylinder_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "soft_cylinder")
        self.tray_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rectangular_tray")
        self.cylinder_geom_ids = self._collect_body_geom_ids(self.cylinder_body_id)
        self.tray_geom_ids = self._collect_body_geom_ids(self.tray_body_id)

    def _collect_body_geom_ids(self, body_id: int) -> set[int]:
        geom_ids: set[int] = set()
        for geom_id in range(self.model.ngeom):
            if self.model.geom_bodyid[geom_id] == body_id:
                geom_ids.add(geom_id)
        return geom_ids

    def capture_rgb_image(self) -> np.ndarray:
        self.renderer.update_scene(self.data, camera=self.camera_id)
        return self.renderer.render()

    def _geom_center_on_table(self, geom_ids: set[int], table_z: float) -> np.ndarray | None:
        self.renderer.enable_segmentation_rendering()
        self.renderer.update_scene(self.data, camera=self.camera_id)
        segmentation = self.renderer.render()

        if segmentation.ndim == 3:
            geom_channel = segmentation[:, :, 0]
        else:
            geom_channel = segmentation

        mask = np.isin(geom_channel, list(geom_ids))
        if not np.any(mask):
            return None

        ys, xs = np.where(mask)
        pixel_x = float(np.mean(xs))
        pixel_y = float(np.mean(ys))
        return self._pixel_to_world(pixel_x, pixel_y, table_z=table_z)

    def _pixel_to_world(self, pixel_x: float, pixel_y: float, table_z: float = 0.40) -> np.ndarray:
        """Project a pixel onto the table plane."""
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

        scale = (table_z - cam_pos[2]) / ray_world[2]
        return cam_pos + scale * ray_world

    def detect(self, table_z: float = 0.40) -> dict:
        rgb_image = self.capture_rgb_image()
        cylinder_position = self._geom_center_on_table(self.cylinder_geom_ids, table_z=table_z)
        tray_position = self._geom_center_on_table(self.tray_geom_ids, table_z=table_z)

        return {
            "image": rgb_image,
            "cylinder_mask": None,
            "tray_mask": None,
            "cylinder_position": cylinder_position,
            "tray_position": tray_position,
        }

    def close(self) -> None:
        self.renderer.close()
