from copy import deepcopy


class Link:
    """Rigid component in a robot. Contains name, visual, collision, and mass."""
    name: str
    visual: str | None = None
    collision: str | None = None
    mass: float | None = None

class Pose:
    """The position and orientation of an object."""
    translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation: tuple[float, float, float] = (0.0, 0.0, 0.0)

class JointCore:
    """Base class for all joints."""
    parent: Link
    child: Link
    pose: Pose
    _subjoints: list['Joint'] = []

class RigidJoint(JointCore):
    """Joint that does not allow translation or rotation."""
    parent: Link
    child: Link
    pose: Pose
    _subjoints: list['Joint'] = []

class RevoluteJoint(JointCore):
    """Joint that allows rotation around a single axis."""
    axis: tuple[float, float, float]
    limits: tuple[float, float] | None = None
    _subjoints: list['Joint'] = []

class SliderJoint(JointCore):
    """Joint that allows translation along a single axis."""
    axis: tuple[float, float, float]
    limits: tuple[float, float]
    _subjoints: list['Joint'] = []

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
        self.base = base
        self.joints = joints
        self.pose = pose

    def _generate_joint_tree(self):
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
        self._generate_joint_tree()  # Regenerate the joint tree after attachment

    def clone_model(self, name: str, pose: Pose = Pose()) -> "Model":
        """Return a deep copy of the model with a new name and pose."""
        joints = deepcopy(self.joints)
        base = deepcopy(self.base)
        copied_model = Model(name=name, joints=joints, base=base, pose=pose)
        return copied_model
