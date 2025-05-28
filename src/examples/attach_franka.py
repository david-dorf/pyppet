from marionette.robots import franka_research_3
import rerun as rr


"""Attaches a Franka Research 3 robot to another Franka Research 3."""

rr.init("", spawn=True)

model1 = franka_research_3  # Creates an instance of the franka_research_3 robot model
model1.visualize()  # Constructs the model tree and visualizes the first robot model
model2 = model1.copy("model2")  # Creates a copy of model1 named model2
model1.attach(model2, "joint6")  # Attaches model2 to joint6 of model1
