from marionette.format import RigidJoint
from marionette.robots import franka_research_3
import time
import math
import rerun as rr


rr.init("", spawn=True)
franka_research_3.visualize()

i=0
while True:
    try:
        for joint_name, joint in franka_research_3.joints.items():
            if not isinstance(joint, RigidJoint):
                limits = joint.limits
                if limits is None:
                    position = i
                else:
                    position = (limits[0] + limits[1]) / 2 + (limits[0] - limits[1]) / 2 * math.sin(i)
                franka_research_3.move_joint(joint_name, position)
        i+=0.01
        time.sleep(0.01)  # Add a small delay to control the speed of the movement
    except KeyboardInterrupt:
        break
