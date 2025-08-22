from pyppet.format import (
    Sphere,
    Box,
    Cylinder,
    Mesh,
    Pose,
    Physics,
    Visual,
    Link,
    Limits,
    RigidJoint,
    RevoluteJoint,
    SliderJoint,
    Model
)

from pyppet.urdf import model_to_urdf


link0 = Link(name = 'link0', visual = Visual(Sphere(radius=0.5)))
link1 = Link(name = 'link1', visual = Visual(Box(width=0.1, height=0.1, depth=0.2)))
link2 = Link(name = 'link2', visual = Visual(Cylinder(radius=0.1, height=0.1)))
link3 = Link(name = 'link3', visual = Visual(Cylinder(radius=0.1, height=0.1),
                                             color = (0.5, 0.6, 0.7)))
link4 = Link(name = 'link4', collision = Cylinder(radius=0.5, height=0.1))
link5 = Link(name = 'link5', physics = Physics(mass = 1,
                                               inertia = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6),
                                               center_of_mass = Pose(translation = (0.1, 0.2, 0.3)),
                                               friction = (0.1)))

joint0 = RigidJoint(
    parent = link0,
    child = link1,
    pose = Pose(translation = (0, 0, 0.333)),
)

joint1 = RevoluteJoint(
    parent = link1,
    child = link2,
    pose = Pose(),
    axis = (0, 1, 0),
    limits = Limits(position_range = (-1.7628, 1.7628)),
)

joint2 = RevoluteJoint(
    parent = link2,
    child = link3,
    pose = Pose(translation = (0, 0, 0.316)),
    axis = (0, 0, 1),
    limits = Limits(position_range = (-2.8973, 2.8973)),
)

joint3 = RevoluteJoint(
    parent = link3,
    child = link4,
    pose = Pose(translation = (0.0825, 0, 0)),
    axis = (0, -1, 0),
    limits = Limits(position_range = (-3.0718, -0.0696)),
)

joint4 = SliderJoint(
    parent = link4,
    child = link5,
    pose = Pose(translation = (-0.0825, 0, 0.384)),
    axis = (0, 0, 1),
    limits = Limits(position_range = (-1, 1)),
)

joints = [joint0, joint1, joint2, joint3, joint4]

MODEL = Model(name = "test_model", joints = joints, base = link0)
model_to_urdf(MODEL, "test_urdf.xml")
