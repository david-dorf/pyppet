import mujoco
import mujoco.viewer
import os
from pathlib import Path
import rerun as rr
from marionette.robots import franka_research_3


pixi_root = Path(os.environ['PIXI_PROJECT_ROOT'])
model = mujoco.MjModel.from_xml_path(str(pixi_root / "src/marionette/models" / "franka_research_3/scene.xml"))
data = mujoco.MjData(model)
# rr.init("", spawn=True)
franka_research_3.visualize()

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        try:
            # print("position", data.qpos)
            # print("control", data.ctrl)
            # print("velocity", data.qvel)
            # print("acceleration", data.qacc)
            # for joint_index in range(model.njnt):
            #     franka_research_3.move_joint(joint_index, data.qpos[joint_index])
            mujoco.mj_step(model, data)
            viewer.sync()
        except KeyboardInterrupt:
            break
