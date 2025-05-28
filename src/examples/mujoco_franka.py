import mujoco
import mujoco.viewer
import os
from pathlib import Path


pixi_root = Path(os.environ['PIXI_PROJECT_ROOT'])
model = mujoco.MjModel.from_xml_path(str(pixi_root / "src/marionette/models" / "franka_fr3/scene.xml"))
data = mujoco.MjData(model)
mujoco.viewer.launch(model, data)
