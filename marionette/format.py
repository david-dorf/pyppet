import rerun as rr
from dataclasses import dataclass
from scipy.spatial.transform import Rotation
import numpy as np
import os
from pathlib import Path
from copy import deepcopy


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
    parent: 'Link'
    child: 'Link'
    pose: 'Pose'

@dataclass
class RevoluteJoint(RigidJoint):
    """Joint that allows rotation around a single axis."""
    axis: tuple[float, float, float]
    limits: tuple[float, float] | None = None

@dataclass
class SliderJoint(RigidJoint):
    """Joint that allows translation along a single axis."""
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
        origin: An optional pose specifying the model translation and rotation.
    """
    def __init__(self, name: str, joints: list[Joint], base: Link, origin: Pose = Pose()):
        self.joints = joints
        self.base_link = base
        self.base_link_path = name + "/" + self.base_link.name
        self.link_path_map = {self.base_link.name: self.base_link_path}
        self.origin = origin
        self.pixi_root = Path(os.environ['PIXI_PROJECT_ROOT'])
        self.asset_path = self.pixi_root / "src" / "marionette" / "models" / name / "assets"
        self.parent_link_to_joints = {}
        for joint in self.joints:
            self.parent_link_to_joints.setdefault(joint.parent.name, []).append(joint)

    def _load_asset(self, rr_path: str, visual_file: str, pose: Pose = Pose()):
        rotation = Rotation.from_euler('xyz', pose.rotation).as_matrix()
        asset = rr.Asset3D(path=f"{self.asset_path}/{visual_file}")
        rr.log(rr_path, asset, rr.Transform3D(translation=pose.translation, mat3x3=rotation))

    def _traverse_joint_tree(self, rr_path: str, current_link_name: str):
        for joint in self.parent_link_to_joints.get(current_link_name, []):
            child_path = rr_path + "/" + joint.child.name
            self.link_path_map[joint.child.name] = child_path
            self._load_asset(child_path, joint.child.visual, joint.transform)
            self._traverse_joint_tree(child_path, joint.child.name)

    def visualize(self):
        """Visualize the model in Rerun."""
        self._load_asset(self.base_link_path, self.base_link.visual, self.origin)
        self._traverse_joint_tree(self.base_link_path, self.base_link.name)

    def attach(self, other_model: "Model", joint_index: int, pose: Pose = Pose()):
        """Attach another model to this model at the specified joint and optional pose."""
        joint = self.joints[joint_index]
        other_model.origin = pose
        other_model.base_link_path = self.link_path_map[joint.child.name] + "/" + other_model.base_link.name
        self.link_path_map.update(other_model.link_path_map)
        other_model.visualize()

    def move_joint(self, joint_index: int, position: float):
        """Move the specified joint to the given position."""
        joint = self.joints[joint_index]
        if isinstance(joint, RigidJoint):
            return
        rotation = Rotation.from_euler('xyz', joint.pose.rotation).as_matrix()
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

    def copy(self, name: str, origin: Pose = Pose()) -> "Model":
        """Return a deep copy of the model with a new name and pose."""
        joints = deepcopy(self.joints)
        base_link = deepcopy(self.base_link)
        copied_model = Model(name=name, joints=joints, base=base_link, origin=origin)
        copied_model.link_path_map = deepcopy(self.link_path_map)
        copied_model.parent_link_to_joints = deepcopy(self.parent_link_to_joints)
        copied_model.asset_path = self.asset_path
        return copied_model
