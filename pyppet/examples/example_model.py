import pyppet.format as pf


# Geometry elements
sphere_0 = pf.Sphere(radius=0.05)
box_0 = pf.Box(length=0.05, width=0.05, height=0.1)
cylinder_0 = pf.Cylinder(radius=0.02, height=0.35)
mesh_0 = pf.Mesh(filename='pyppet/examples/example_mesh.stl', scale=(0.8, 0.9, 0.7))
mesh_1 = pf.Mesh(filename='pyppet/examples/example_mesh.stl', scale=(0.96, 1.2, 1))

# Color elements
color_orange = (0.9, 0.5, 0.1)
color_blue = (0, 0, 0.5)

# Visual elements
visual_0 = pf.Visual(sphere_0)
visual_1 = pf.Visual(box_0, color_blue)
visual_2 = pf.Visual(cylinder_0, color_orange)
visual_3 = pf.Visual(mesh_0, color_orange)
visual_4 = pf.Visual(mesh_1, color_blue)

# Physics elements
physics_0 = pf.Physics(
    mass = 3,
    inertia = (1e-6, 1e-6, 1e-6, 0.0, 0.0, 0.0),
    center_of_mass = pf.Pose(translation = (0.1, 0.2, 0.3)),
    friction = (0.1)
)

base_link = pf.BaseLink()
link0 = pf.Link(name = 'link0', visual = visual_0)
link1 = pf.Link(name = 'link1', visual = visual_1)
link2 = pf.Link(name = 'link2', visual = visual_2, collision = sphere_0)
link3 = pf.Link(name = 'link3', visual = visual_3, collision = box_0, physics = physics_0)
link4 = pf.Link(name = 'link4', visual = visual_4, collision = mesh_1)
link5 = pf.Link(name = 'link5', physics = physics_0)
link6 = pf.Link(name = 'link6')
link7 = link3.copy_link(new_name='link7')
link7.name = 'link7'

base_joint = pf.RigidJoint(
    parent = base_link,
    child = link0,
)

joint0 = pf.RigidJoint(
    parent = link0,
    child = link1,
    pose = pf.Pose(translation = (0, 0, 0.333), rotation = (0.2, 0.871, 10.0)),
)

joint1 = pf.RevoluteJoint(
    parent = link1,
    child = link2,
    pose = pf.Pose(),
    axis = (0, 1, 0),
    limits = pf.Limits(position_range = (-1.7628, 1.7628)),
)

joint2 = pf.RevoluteJoint(
    parent = link2,
    child = link3,
    pose = pf.Pose(translation = (0, 0, 0.316), rotation = (0.2, -0.2, 0.01)),
    axis = (0, 0, 1),
    limits = pf.Limits(position_range = (-2.8973, 2.8973)),
)

joint3 = pf.RevoluteJoint(
    parent = link3,
    child = link4,
    pose = pf.Pose(translation = (0.0825, 0, 0)),
    axis = (0, -1, 0),
    limits = pf.Limits(position_range = (-3.0718, -0.0696)),
)

joint4 = pf.SliderJoint(
    parent = link4,
    child = link5,
    pose = pf.Pose(translation = (-0.0825, 0, 0.384)),
    axis = (0, 0, 1),
    limits = pf.Limits(position_range = (-1, 1)),
)

joint5 = pf.RigidJoint(
    parent = link5,
    child = link6,
)

joint6 = pf.RigidJoint(
    parent = link5,
    child = link7,
    pose = pf.Pose(translation = (2, 0.2, 0)),
)

joints = [base_joint, joint0, joint1, joint2, joint3, joint4, joint5, joint6]

EXAMPLE_MODEL = pf.Model(name = "example_model", joints = joints)
