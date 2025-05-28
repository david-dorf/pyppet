from marionette.robots import franka_research_3
from marionette.format import Transform
import rerun as rr


"""Spawns multiple Franka Research 3 robots in a grid layout."""

rr.init("", spawn=True)

cols = 5
rows = 5
spacing = 1.0
for i in range(rows * cols):
    row = i // cols
    col = i % cols
    new_model = franka_research_3.copy(f"franka_{i}", Transform(x=row, y=col))
    new_model.visualize()
