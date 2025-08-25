from dataclasses import dataclass


@dataclass
class Sphere:
    radius: float

    def __post_init__(self):
        if self.radius <= 0:
            raise ValueError(f"Invalid radius: {self.radius}. Radius must be positive.")


@dataclass
class Box:
    width: float
    height: float
    depth: float

    def __post_init__(self):
        if self.width <= 0:
            raise ValueError(f"Invalid width: {self.width}. Width must be positive.")
        if self.height <= 0:
            raise ValueError(f"Invalid height: {self.height}. Height must be positive.")
        if self.depth <= 0:
            raise ValueError(f"Invalid depth: {self.depth}. Depth must be positive.")


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
    geometry: Geometry | None = None
    color: tuple[float, float, float] | None = None


@dataclass
class Link:
    """Rigid component in a robot. Contains name, visual, collision, and physical properties."""
    name: str
    visual: Visual | None = None
    collision: Geometry | None = None
    physics: Physics | None = None


@dataclass
class Limits:
    position_range: tuple[float, float] | None = None
    velocity: float | None = None
    force: float | None = None


class RigidJoint:
    """Connection that does not allow translation or rotation between parent and child links."""
    def __init__(self, parent: Link, child: Link, pose: Pose):
        self.parent = parent
        self.child = child
        self.pose = pose
        self._subjoints: list['Joint'] = []


class MobileJoint(RigidJoint):
    """Base class for joints that allow translation or rotation between parent and child links."""
    def __init__(self, parent: Link, child: Link, pose: Pose, axis: tuple[float, float, float], limits: Limits | None = None, friction: float | None = None, damping: float | None = None):
        super().__init__(parent, child, pose)
        self.axis = axis
        self._subjoints: list['Joint'] = []
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
        base: The first link in the model kinematic chain.
        pose: An optional pose specifying the model translation and rotation.
    """
    def __init__(self, name: str, joints: JointList, base: Link, pose: Pose = Pose()):
        self.name = name
        self.joints = joints
        self.base = base
        self.pose = pose
        self.links = self._generate_link_list()
        self._generate_joint_tree()

    def _generate_joint_tree(self):
        """Appends subjoints to joints when a joint child is the same as another joint parent."""
        child_to_joint_map = {}
        for joint in self.joints:
            child_to_joint_map[joint.child.name] = joint
        for joint in self.joints:
            if joint.parent.name in child_to_joint_map:
                if joint.parent.name == child_to_joint_map[joint.parent.name].child.name:
                    joint._subjoints.append(joint)

    def _generate_link_list(self) -> list[Link]:
        """Generates a list of links in the model."""
        link_list = [self.base]
        for joint in self.joints:
            link_list.append(joint.child)
        return link_list

    def attach_model(self, other_model: "Model", joint: Joint, pose: Pose = Pose()):
        """Attach another model to this model at the specified joint and optional pose."""
        joint.child = other_model.base
        other_model.pose = pose
        self._generate_joint_tree()
