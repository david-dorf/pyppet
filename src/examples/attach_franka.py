from marionette.format import Model
from marionette.robots import franka_research_3
import copy
import rerun as rr


rr.init("", spawn=True)
model1 = Model("franka_panda", franka_research_3.joints)
model2 = Model("franka_panda2", copy.deepcopy(franka_research_3.joints))
model1.visualize()
model1.attach(model2, "joint6")
