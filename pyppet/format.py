class Sphere:
    radius: float

class Box:
    width: float
    height: float
    depth: float

class Cylinder:
    radius: float
    height: float

class Mesh:
    filename: str

Geometry = Sphere | Box | Cylinder | Mesh

class Pose:
    """The position and orientation of an object."""
    translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation: tuple[float, float, float] = (0.0, 0.0, 0.0)

class Physics:
    """Physical properties of an object. Inertia is in the order of xx, yy, zz, xy, xz, yz."""
    mass: float | None = None
    inertia: tuple[float, float, float, float, float, float] | None = None
    center_of_mass: Pose | None = None
    friction: float | None = None

class Visual:
    """Visual properties of an object. Color is in the order of red, green, blue [0.0 to 1.0]."""
    geometry: Geometry | None = None
    color: tuple[float, float, float] | None = None

class Link:
    """Rigid component in a robot. Contains name, visual, collision, and physical properties."""
    name: str
    visual: Visual | None = None
    collision: Geometry | None = None
    physics: Physics | None = None

class RigidJoint:
    """Connection that does not allow translation or rotation between parent and child links."""
    parent: Link
    child: Link
    pose: Pose
    _subjoints: list['Joint'] = []

class MobileJoint(RigidJoint):
    """Base class for joints that allow translation or rotation between parent and child links."""
    axis: tuple[float, float, float]
    limits: tuple[float, float] | None = None
    friction: float | None = None
    damping: float | None = None

class RevoluteJoint(MobileJoint):
    """Joint for rotation around an axis. The rotation is continuous when limits are None."""

class SliderJoint(MobileJoint):
    """Joint for translation along an axis."""

Joint = RigidJoint | RevoluteJoint | SliderJoint

class Model:
    """
    Defines a robot model.

    Attributes:
        name: The name of the model.
        joints: A list of joints that the model is composed of.
        base: The first link in the model kinematic chain.
        pose: An optional pose specifying the model translation and rotation.
    """
    def __init__(self, name: str, joints: list[Joint], base: Link, pose: Pose = Pose()):
        self.name = name
        self.joints = joints
        self.base = base
        self.pose = pose

    def _generate_joint_tree(self):
        """Appends subjoints when a joint child is the same as another joint parent."""
        joint_to_parent_map = {}
        for joint in self.joints:
            joint_to_parent_map[joint] = joint.parent
        for joint in self.joints:
            if joint.child == joint_to_parent_map[joint]:
                joint._subjoints.append(joint)

    def attach_model(self, other_model: "Model", joint: Joint, pose: Pose = Pose()):
        """Attach another model to this model at the specified joint and optional pose."""
        joint.child = other_model.base
        other_model.pose = pose
        self._generate_joint_tree()
