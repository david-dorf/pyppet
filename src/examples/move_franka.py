from marionette.robots import franka_research_3
from marionette.format import RigidJoint
import rerun as rr
import time
import math


"""Moves non-rigid joints of a Franka Research 3 robot in a sinusoidal trajectory between limits."""

rr.init("", spawn=True)

i=0
franka_research_3.visualize()
while True:
    try:
        for joint_name, joint in franka_research_3.joints.items():
            if isinstance(joint, RigidJoint):
                continue
            if joint.limits is not None:
                midline = (joint.limits[0] + joint.limits[1]) / 2
                amplitude = (joint.limits[0] - joint.limits[1]) / 2
                position = midline + amplitude * math.sin(i)
                franka_research_3.move_joint(joint_name, position)
        i+=0.01
        time.sleep(0.01)  # Add a small delay to control the speed of the movement
    except KeyboardInterrupt:
        break
