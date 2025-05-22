from marionette.format import Link, RigidJoint, RevoluteJoint, SliderJoint, Transform, Model
from math import pi


link0 = Link(name = 'link0', file = "link0.dae")
link1 = Link(name = 'link1', file = "link1.dae")
link2 = Link(name = 'link2', file = "link2.dae")
link3 = Link(name = 'link3', file = "link3.dae")
link4 = Link(name = 'link4', file = "link4.dae")
link5 = Link(name = 'link5', file = "link5.dae")
link6 = Link(name = 'link6', file = "link6.dae")
link7 = Link(name = 'link7', file = "link7.dae")
hand = Link(name = 'hand', file = "hand.dae")
finger1 = Link(name = 'finger1', file = "finger.dae")
finger2 = Link(name = 'finger2', file = "finger.dae")

joints = {
    "joint0": RevoluteJoint(
        parent = link0,
        child = link1,
        transform = Transform(z = 0.333),
        axis = (0, 0, 1),
        limits = (-2.8973, 2.8973),
    ),

    "joint1": RevoluteJoint(
        parent = link1,
        child = link2,
        transform = Transform(roll = -pi/2),
        axis = (0, 0, 1),
        limits = (-1.7628, 1.7628)),

    "joint2": RevoluteJoint(
        parent = link2,
        child = link3,
        transform = Transform(y = -0.316, roll = pi/2),
        axis = (0, 0, 1),
        limits = (-2.8973, 2.8973)
    ),

    "joint3": RevoluteJoint(
        parent = link3,
        child = link4,
        transform = Transform(x = 0.0825, roll = pi/2),
        axis = (0, 0, 1),
        limits = (-3.0718, -0.0698)
    ),

    "joint4": RevoluteJoint(
        parent = link4,
        child = link5,
        transform = Transform(x = -0.0825, y = 0.384, roll = -pi/2),
        axis = (0, 0, 1),
        limits = (-2.8973, 2.8973)
    ),

    "joint5": RevoluteJoint(
        parent = link5,
        child = link6,
        transform = Transform(roll = pi/2),
        axis = (0, 0, 1),
        limits = (-3.7525, -0.0175)
    ),

    "joint6": RevoluteJoint(
        parent = link6,
        child = link7,
        transform = Transform(x = 0.088, roll = pi/2),
        axis = (0, 0, 1),
        limits = (-2.8973, 2.8973)
    ),

    "joint_hand": RigidJoint(
        parent = link7,
        child = hand,
        transform = Transform(z = 0.107)
    ),

    "joint_finger1": SliderJoint(
        parent = hand,
        child = finger1,
        transform = Transform(z = 0.0584),
        axis = (0, 1, 0),
        limits = (0, 0.04)
    ),

    "joint_finger2": SliderJoint(
        parent = hand,
        child = finger2,
        transform = Transform(z = 0.0584, yaw = pi),
        axis = (0, 1, 0),
        limits = (0, 0.04)
    )
}

franka_research_3 = Model("franka_research_3", joints, link0)
