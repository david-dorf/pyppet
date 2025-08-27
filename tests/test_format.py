from pyppet.examples.example_model import EXAMPLE_MODEL
import pyppet.format as pf


example = EXAMPLE_MODEL

parent_order = ["base_link", "link1", "link2", "link3", "link4", "link5"]
child_order = ["link1", "link2", "link3", "link4", "link5", "link6", "link7"]
visual_geometry_order = [pf.Sphere, pf.Box, pf.Cylinder, pf.Mesh, pf.Mesh, None, None]
collision_geometry_order = [None, None, pf.Sphere, pf.Box, pf.Mesh, None, None]

def test_joint_parent_names():
    """Test joint parent names align with expected order."""
    joint_tree_generator = example.traverse_joint_tree(example.joints[0])
    for parent_name, joint in zip(parent_order, joint_tree_generator):
        assert joint.parent.name == parent_name

def test_joint_child_names():
    """Test joint child names align with expected order."""
    joint_tree_generator = example.traverse_joint_tree(example.joints[0])
    for child_name, joint in zip(child_order, joint_tree_generator):
        assert joint.child.name == child_name

def test_visual_geometry_types():
    """Test visual geometry types align with expected order."""
    for geometry_type, link in zip(visual_geometry_order, example.links):
        if link.visual is None:
            continue
        assert type(link.visual.geometry) == geometry_type

def test_collision_geometry_types():
    """Test collision geometry types align with expected order."""
    for geometry_type, link in zip(collision_geometry_order, example.links):
        if link.collision is None:
            continue
        assert type(link.collision) == geometry_type

def test_physics():
    """Test physics properties are of the correct type."""
    for link in example.links:
        if link.physics is None:
            continue
        assert type(link.physics) == pf.Physics

def test_negative_mass():
    """Test negative mass raises ValueError."""
    try:
        pf.Physics(mass=-1)
        assert False
    except ValueError:
        assert True

def test_negative_inertia():
    """Test negative inertia raises ValueError."""
    try:
        pf.Physics(inertia=(-1, 0, 0, 0, 0, 0))
        assert False
    except ValueError:
        assert True

def test_negative_dimensions():
    """Test negative dimensions raise ValueError."""
    try:
        pf.Box(width = -1, height = -1, depth = -1)
        pf.Sphere(radius = -1)
        pf.Cylinder(radius = -1, height = -1)
        pf.Mesh(filename = "test", scale = (-1, -1, -1))
        assert False
    except ValueError:
        assert True

def test_copy_ids():
    """Ensure the link IDs of a copy are different than the original link IDs"""
    model1 = EXAMPLE_MODEL
    model2 = model1.copy_model()
    for model1_link, model2_link in zip(model1.links, model2.links):
        assert model1_link._id != model2_link._id

def test_attach_model():
    """Test attaching a model to a joint and traversing the joint tree."""
    model1 = EXAMPLE_MODEL  # Creates an instance of the franka_research_3 robot model
    model2 = model1.copy_model()  # Creates a copy of model1 named model2
    try:
        model1.attach_model(model2, model1.joints[5])  # Attaches model to joint 5 in joints array
        model1.traverse_joint_tree(model1.joints[0])
        assert True
    except RecursionError:
        assert False
