from marionette.format import Link, RigidJoint, RevoluteJoint, SliderJoint, Transform, Model
from math import pi


link0 = Link(name = 'link0', visual = "visual/link0.glb")
link1 = Link(name = 'link1', visual = "visual/link1.glb")
link2 = Link(name = 'link2', visual = "visual/link2.glb")
link3 = Link(name = 'link3', visual = "visual/link3.glb")
link4 = Link(name = 'link4', visual = "visual/link4.glb")
link5 = Link(name = 'link5', visual = "visual/link5.glb")
link6 = Link(name = 'link6', visual = "visual/link6.glb")
link7 = Link(name = 'link7', visual = "visual/link7.glb")
hand = Link(name = 'hand', visual = "visual/hand.glb")
finger1 = Link(name = 'finger1', visual = "visual/finger.glb")
finger2 = Link(name = 'finger2', visual = "visual/finger.glb")

joint0 = RevoluteJoint(
    parent = link0,
    child = link1,
    transform = Transform(z = 0.333),
    axis = (0, 0, 1),
    limits = (-2.8973, 2.8973),
)

joint1 = RevoluteJoint(
    parent = link1,
    child = link2,
    transform = Transform(),
    axis = (0, 1, 0),
    limits = (-1.7628, 1.7628),
)

joint2 = RevoluteJoint(
    parent = link2,
    child = link3,
    transform = Transform(z = 0.316),
    axis = (0, 0, 1),
    limits = (-2.8973, 2.8973),
)

joint3 = RevoluteJoint(
    parent = link3,
    child = link4,
    transform = Transform(x = 0.0825),
    axis = (0, -1, 0),
    limits = (-3.0718, -0.0696),
)

joint4 = RevoluteJoint(
    parent = link4,
    child = link5,
    transform = Transform(x = -0.0825, z = 0.384),
    axis = (0, 0, 1),
    limits = (-2.8973, 2.8973),
)

joint5 = RevoluteJoint(
    parent = link5,
    child = link6,
    transform = Transform(),
    axis = (0, -1, 0),
    limits = (-0.0175, 3.7525),
)

joint6 = RevoluteJoint(
    parent = link6,
    child = link7,
    transform = Transform(x = 0.088),
    axis = (0, 0, 1),
    limits = (-2.8973, 2.8973)
)

joint7 = RigidJoint(
    parent = link7,
    child = hand,
    transform = Transform(z = -0.107)
)

joint8 = SliderJoint(
    parent = hand,
    child = finger1,
    transform = Transform(z = -0.0584),
    axis = (0, 1, 0),
    limits = (0, -0.04)
)

joint9 = SliderJoint(
    parent = hand,
    child = finger2,
    transform = Transform(z = -0.0584, yaw = pi),
    axis = (0, 1, 0),
    limits = (0, -0.04)
)

joints = [joint0, joint1, joint2, joint3, joint4, joint5, joint6, joint7, joint8, joint9]
franka_research_3 = Model(name = "franka_research_3", joints = joints, base = link0)
