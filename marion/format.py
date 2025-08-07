from .rotation import rotation_matrix_from_euler
import rerun as rr
from dataclasses import dataclass
from copy import deepcopy
import numpy as np


@dataclass
class Link:
    """Rigid component in a robot. Contains name, visual asset file, and optional collision/mass."""
    name: str
    visual: str
    collision: str | None = None
    mass: float | None = None

@dataclass
class Pose:
    """The position and orientation of an object."""
    translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rotation: tuple[float, float, float] = (0.0, 0.0, 0.0)

@dataclass
class RigidJoint:
    """Joint that does not allow translation or rotation. Base class for all other joints."""
    parent: Link
    child: Link
    pose: Pose

@dataclass
class RevoluteJoint:
    """Joint that allows rotation around a single axis."""
    parent: Link
    child: Link
    pose: Pose
    axis: tuple[float, float, float]
    limits: tuple[float, float] | None = None

@dataclass
class SliderJoint:
    """Joint that allows translation along a single axis."""
    parent: Link
    child: Link
    pose: Pose
    axis: tuple[float, float, float]
    limits: tuple[float, float]

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
    def __init__(self, name: str, joints: list[Joint], base: Link, path: str, pose: Pose = Pose()):
        self.joints = joints
        self.base_link = base
        self.path = path
        self.pose = pose
        self.base_link_id = name + "/" + self.base_link.name  # For Rerun logging tree structure
        self.link_path_map = {self.base_link.name: self.base_link_id}
        self.parent_link_to_joints = {}
        for joint in self.joints:
            self.parent_link_to_joints.setdefault(joint.parent.name, []).append(joint)

    def _load_asset(self, rr_path: str, visual_file: str, pose: Pose = Pose()):
        rotation = rotation_matrix_from_euler(pose.rotation)
        asset = rr.Asset3D(path=f"{self.path}/{visual_file}")
        rr.log(rr_path, asset, rr.Transform3D(translation=pose.translation, mat3x3=rotation))

    def _traverse_joint_tree(self, rr_path: str, current_link_name: str):
        for joint in self.parent_link_to_joints.get(current_link_name, []):
            child_path = rr_path + "/" + joint.child.name
            self.link_path_map[joint.child.name] = child_path
            self._load_asset(child_path, joint.child.visual, joint.pose)
            self._traverse_joint_tree(child_path, joint.child.name)

    def visualize(self):
        """Visualize the model in Rerun."""
        self._load_asset(self.base_link_id, self.base_link.visual, self.pose)
        self._traverse_joint_tree(self.base_link_id, self.base_link.name)

    def attach(self, other_model: "Model", joint_index: int, pose: Pose = Pose()):
        """Attach another model to this model at the specified joint and optional pose."""
        joint = self.joints[joint_index]
        other_model.pose = pose
        other_model.base_link_id = self.link_path_map[joint.child.name] + "/" + other_model.base_link.name
        self.link_path_map.update(other_model.link_path_map)
        other_model.visualize()

    def move_joint(self, joint: Joint, position: float):
        """Move the specified non-rigid joint to the given position."""
        if isinstance(joint, RigidJoint):
            return
        rotation = rotation_matrix_from_euler(joint.pose.rotation)
        axis_unit_vector = (rotation @ joint.axis) / np.linalg.norm(rotation @ joint.axis)
        rr_path = self.link_path_map[joint.child.name]
        if joint.limits is not None:
            min_limit, max_limit = min(joint.limits), max(joint.limits)
            if not (min_limit <= position <= max_limit):
                return
        if isinstance(joint, SliderJoint):
            translation = axis_unit_vector * position + joint.pose.translation
            rr.log(rr_path, rr.Transform3D(translation=translation, clear=False))
        elif isinstance(joint, RevoluteJoint):
            rotation = rr.RotationAxisAngle(axis=axis_unit_vector, radians=position)
            rr.log(rr_path, rr.Transform3D(rotation=rotation, clear=False))

    def copy(self, name: str, pose: Pose = Pose()) -> "Model":
        """Return a deep copy of the model with a new name and pose."""
        joints = deepcopy(self.joints)
        base_link = deepcopy(self.base_link)
        path = deepcopy(self.path)
        copied_model = Model(name=name, joints=joints, base=base_link, path=path, pose=pose)
        copied_model.link_path_map = deepcopy(self.link_path_map)
        copied_model.parent_link_to_joints = deepcopy(self.parent_link_to_joints)
        return copied_model
