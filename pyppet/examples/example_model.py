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


# Geometry elements
sphere_0 = Sphere(radius=0.05)
box_0 = Box(width=0.05, height=0.05, depth=0.1)
cylinder_0 = Cylinder(radius=0.02, height=0.35)
mesh_0 = Mesh(filename='pyppet/examples/example_mesh.stl', scale=(0.005, 0.001, 0.0312343575))
mesh_1 = Mesh(filename='pyppet/examples/example_mesh.stl', scale=(0.002, 0.002, 1e-6))
color_orange = (0.9, 0.5, 0.1)
color_blue = (0, 0, 0.5)

# Visual elements
visual_0 = Visual(sphere_0)
visual_1 = Visual(box_0, color_blue)
visual_2 = Visual(cylinder_0, color_orange)
visual_3 = Visual(mesh_0, color_orange)
visual_4 = Visual(mesh_1, color_blue)

# Physics elements
physics_0 = Physics(
    mass = 3,
    inertia = (1e-6, 1e-6, 1e-6, 0.0, 0.0, 0.0),
    center_of_mass = Pose(translation = (0.1, 0.2, 0.3)),
    friction = (0.1)
)

link0 = Link(name = 'link0', visual = visual_0)
link1 = Link(name = 'link1', visual = visual_1)
link2 = Link(name = 'link2', visual = visual_2)
link3 = Link(name = 'link3', visual = visual_3)
link4 = Link(name = 'link4', visual = visual_4, collision = mesh_1)
link5 = Link(name = 'link5', physics = physics_0)

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

EXAMPLE_MODEL = Model(name = "example_model", joints = joints, base = link0)
