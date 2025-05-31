import mujoco
import mujoco.viewer
import os
from pathlib import Path


pixi_root = Path(os.environ['PIXI_PROJECT_ROOT'])
model = mujoco.MjModel.from_xml_path(str(pixi_root / "src/marionette/models" / "franka_research_3/scene.xml"))
data = mujoco.MjData(model)
with mujoco.viewer.launch_passive(model, data) as viewer:
  while viewer.is_running():
    print("position", data.qpos)
    print("control", data.ctrl)
    print("velocity", data.qvel)
    print("acceleration", data.qacc)
    mujoco.mj_step(model, data)
    viewer.sync()
