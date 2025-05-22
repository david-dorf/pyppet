from marionette.robots import franka_research_3
import rerun as rr


rr.init("", spawn=True)

model1 = franka_research_3
model2 = franka_research_3.copy("model2")
model1.attach(model2, "joint6")
model1.visualize()
