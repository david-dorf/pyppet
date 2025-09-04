from dataclasses import dataclass
from copy import deepcopy
import uuid


@dataclass
class Sphere:
    radius: float

    def __post_init__(self):
        if self.radius <= 0:
            raise ValueError(f"Invalid radius: {self.radius}. Radius must be positive.")


@dataclass
class Box:
    length: float
    width: float
    height: float

    def __post_init__(self):
        if self.length <= 0:
            raise ValueError(f"Invalid length: {self.length}. Length must be positive.")
        if self.width <= 0:
            raise ValueError(f"Invalid width: {self.width}. Width must be positive.")
        if self.height <= 0:
            raise ValueError(f"Invalid height: {self.height}. Height must be positive.")


@dataclass
class Cylinder:
    radius: float
    height: float

    def __post_init__(self):
        if self.radius <= 0:
            raise ValueError(f"Invalid radius: {self.radius}. Radius must be positive.")
        if self.height <= 0:
            raise ValueError(f"Invalid height: {self.height}. Height must be positive.")


@dataclass
class Mesh:
    filename: str
    scale: tuple[float, float, float] = (1.0, 1.0, 1.0)

    def __post_init__(self):
        if any(s <= 0 for s in self.scale):
            raise ValueError(f"Invalid scale: {self.scale}. All scale values must be positive.")


Geometry = Sphere | Box | Cylinder | Mesh


@dataclass
class Pose:
    """The position and orientation of an object."""
    translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass
class Physics:
    """Physical properties of an object. Inertia is in the order of xx, yy, zz, xy, xz, yz."""
    mass: float | None = None
    inertia: tuple[float, float, float, float, float, float] | None = None
    center_of_mass: Pose | None = None
    friction: float | None = None

    def __post_init__(self):
        if self.mass is not None and self.mass <= 0:
            raise ValueError(f"Invalid mass: {self.mass}. Mass must be positive.")

        if self.inertia is not None:
            if any(i < 0 for i in self.inertia):
                raise ValueError(f"Invalid inertia: {self.inertia}. All inertia values must be positive.")


@dataclass
class Visual:
    """Visual properties of an object. Color is in the order of red, green, blue [0.0 to 1.0]."""
    geometry: Geometry
    color: tuple[float, float, float] | None = (0.0, 0.0, 0.0)

@dataclass
class Limits:
    position_range: tuple[float, float] | None = None
    velocity: float | None = None
    force: float | None = None

class Link:
    """Rigid component in a robot. Contains name, visual, collision, and physical properties."""
    def __init__(self, name: str, visual: Visual | None = None, collision: Geometry | None = None, physics: Physics | None = None):
        self.name = name
        self._id = uuid.uuid4()
        self.visual = visual
        self.collision = collision
        self.physics = physics

    def copy_link(self):
        link_copy = deepcopy(self)
        link_copy._id = uuid.uuid4()
        return link_copy

class BaseLink(Link):
    """First link in a robot that serves as the root of the kinematic tree."""
    def __init__(self, name = "base_link", visual = None, collision = None, physics = None):
        super().__init__(name, visual, collision, physics)

class RigidJoint:
    """Connection that does not allow translation or rotation between parent and child links."""
    def __init__(self, parent: Link, child: Link, pose: Pose = Pose()):
        self.parent = parent
        self.child = child
        self.pose = pose
        self._parent_joint: Joint | None = None

class MobileJoint(RigidJoint):
    """Base class for joints that allow translation or rotation between parent and child links."""
    def __init__(self, parent: Link, child: Link, pose: Pose, axis: tuple[float, float, float], limits: Limits | None = None, friction: float | None = None, damping: float | None = None):
        super().__init__(parent, child, pose)
        self.axis = axis
        if limits is not None:
            self.position_range = limits.position_range
            self.velocity_limit = limits.velocity
            self.force_limit = limits.force
        self.friction = friction
        self.damping = damping
        self._position = 0.0

    def set_position(self, position: float):
        self._position = position

    def get_position(self) -> float:
        return self._position


class RevoluteJoint(MobileJoint):
    """Joint for rotation around an axis."""


class SliderJoint(MobileJoint):
    """Joint for translation along an axis."""


Joint = RigidJoint | RevoluteJoint | SliderJoint
JointList = list[Joint] | list[RigidJoint] | list[RevoluteJoint] | list[SliderJoint]


class Model:
    """
    Defines a robot model.

    Attributes:
        name: The name of the model.
        joints: A list of joints that the model is composed of.
        pose: An optional pose specifying the model translation and rotation.
        base_link: The first link in the model kinematic chain.
        base_joint: The first joint in the model kinematic chain.
        links: A list of links that the model is composed of.
    """
    def __init__(self, name: str, joints: JointList, pose: Pose = Pose()):
        self.name = name
        self.joints = joints
        self.pose = pose
        self.base_link, self.base_joint = self._get_base()
        self.links = self._generate_link_list()
        self._generate_joint_tree()

    def _get_base(self) -> tuple[Link, Joint]:
        base_links = []
        base_joint = RigidJoint(Link(name='default'), Link(name='default'))  # Prevents unbounded
        for joint in self.joints:
            if type(joint.parent) is BaseLink:
                base_links.append(joint.parent)
                base_joint = joint
        if len(base_links) == 1:
            return base_links[0], base_joint
        elif len(base_links) > 1:
            raise ValueError("Multiple base links found")
        raise ValueError("No base link found")

    def _generate_link_list(self) -> list[Link]:
        """Generates a list of links in the model."""
        link_list = [self.base_link]
        for joint in self.joints:
            link_list.append(joint.child)
        return link_list

    def _generate_joint_tree(self):
        """Constructs a joint tree using information on parent and child links."""
        child_id_to_joint_map: dict[uuid.UUID, Joint] = {self.base_link._id: self.base_joint}
        for joint in self.joints:
            child_id_to_joint_map[joint.child._id] = joint
        for joint in self.joints:
            if joint.parent._id not in child_id_to_joint_map:
                continue
            if joint.parent._id == child_id_to_joint_map[joint.parent._id].child._id:
                joint._parent_joint = child_id_to_joint_map[joint.parent._id]

    def traverse_joint_tree(self, joint: Joint):
        """Traverses the joint tree starting from the specified joint to the root."""
        yield joint
        if joint._parent_joint is not None:
            yield from self.traverse_joint_tree(joint._parent_joint)

    def attach_model(self, other_model: "Model", joint: Joint, pose: Pose = Pose()):
        """Attach another model to this model at the specified joint and optional pose."""
        joint.child = other_model.base_link
        other_model.pose = pose
        self._generate_joint_tree()

    def copy_model(self):
        """Returns a deep copy of the model with new unique link IDs."""
        copied_model = deepcopy(self)
        for link in copied_model.links:
            link._id = uuid.uuid4()
        return copied_model
