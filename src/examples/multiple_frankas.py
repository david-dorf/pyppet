from marionette.format import Model, Transform
from marionette.models.franka_research_3.franka_research_3 import franka_research_3
import copy
import rerun as rr


rr.init("", spawn=True)
cols = 5
rows = 5
spacing = 1.0
for i in range(rows * cols):
    row = i // cols
    col = i % cols
    new_model = Model(f"franka_panda_{i}",
                      copy.deepcopy(franka_research_3.joints),
                      Transform(x = col * spacing, y = row * spacing))
    new_model.visualize()
