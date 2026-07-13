"""Simulation environment setup."""

from pathlib import Path

import mujoco

ASSETS_DIR = Path(__file__).resolve().parents[1] / "assets"
SCENE_PATH = ASSETS_DIR / "scene.xml"


class SimulationEnvironment:
    """Loads the MuJoCo scene and exposes the model and data."""

    def __init__(self, scene_path: Path | None = None):
        self.scene_path = scene_path or SCENE_PATH
        self.model = mujoco.MjModel.from_xml_path(str(self.scene_path))
        self.data = mujoco.MjData(self.model)
        self._reset_scene()

    def _reset_scene(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        if self.model.nkey > 0:
            arm_joint_count = 9
            key_qpos = self.model.key_qpos[0][:arm_joint_count]
            key_ctrl = self.model.key_ctrl[0].copy()
            self.data.qpos[:arm_joint_count] = key_qpos
            self.data.ctrl[: len(key_ctrl)] = key_ctrl
            free_joint_start = arm_joint_count
            self.data.qpos[free_joint_start : free_joint_start + 7] = self.model.qpos0[
                free_joint_start : free_joint_start + 7
            ]
        mujoco.mj_forward(self.model, self.data)

    @property
    def physics_timestep(self) -> float:
        return self.model.opt.timestep

    def step(self, count: int = 1) -> None:
        for _ in range(count):
            mujoco.mj_step(self.model, self.data)

    def forward(self) -> None:
        mujoco.mj_forward(self.model, self.data)

    def body_position(self, body_name: str):
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        return self.data.xpos[body_id].copy()

    def site_position(self, site_name: str):
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        return self.data.site_xpos[site_id].copy()
