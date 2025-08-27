from pyppet.examples.example_model import EXAMPLE_MODEL
import pyppet.format as pf

example = EXAMPLE_MODEL

parent_order = ["base_link", "link1", "link2", "link3", "link4", "link5"]
child_order = ["link1", "link2", "link3", "link4", "link5", "link6", "link7"]
visual_geometry_order = [pf.Sphere, pf.Box, pf.Cylinder, pf.Mesh, pf.Mesh, None, None]
collision_geometry_order = [None, None, pf.Sphere, pf.Box, pf.Mesh, None, None]

def test_joint_parent_names():
    joint_tree_generator = example.traverse_joint_tree(example.joints[0])
    for parent_name, joint in zip(parent_order, joint_tree_generator):
        assert joint.parent.name == parent_name

def test_joint_child_names():
    joint_tree_generator = example.traverse_joint_tree(example.joints[0])
    for child_name, joint in zip(child_order, joint_tree_generator):
        assert joint.child.name == child_name

def test_visual_geometry_types():
    for geometry_type, link in zip(visual_geometry_order, example.links):
        if link.visual is None:
            continue
        assert type(link.visual.geometry) == geometry_type

def test_collision_geometry_types():
    for geometry_type, link in zip(collision_geometry_order, example.links):
        if link.collision is None:
            continue
        assert type(link.collision) == geometry_type

def test_physics():
    for link in example.links:
        if link.physics is None:
            continue
        assert type(link.physics) == pf.Physics

def test_negative_mass():
    try:
        pf.Physics(mass=-1)
        assert False
    except ValueError:
        assert True

def test_negative_inertia():
    try:
        pf.Physics(inertia=(-1, 0, 0, 0, 0, 0))
        assert False
    except ValueError:
        assert True

def test_negative_dimensions():
    try:
        pf.Box(width = -1, height = -1, depth = -1)
        pf.Sphere(radius = -1)
        pf.Cylinder(radius = -1, height = -1)
        pf.Mesh(filename = "test", scale = (-1, -1, -1))
        assert False
    except ValueError:
        assert True
