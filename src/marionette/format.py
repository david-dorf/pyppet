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
class Transform:
    """Translation in meters and rotation in radians of a rigid body."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    def translation(self): return self.x, self.y, self.z
    def rotation(self): return self.roll, self.pitch, self.yaw

@dataclass
class RigidJoint:
    """Joint that does not allow translation or rotation.

    Attributes:
        parent: The parent link of the joint.
        child: The child link of the joint.
        transform: The translation and rotation of the child link from the parent link.
    """
    parent: Link
    child: Link
    transform: Transform

@dataclass
class RevoluteJoint:
    """
    Joint that allows rotation around a single axis.
    Limits are optional and the joint will rotate continuously without limits.

    Attributes:
        parent: The parent link of the joint.
        child: The child link of the joint.
        transform: The translation and rotation of the child link from the parent link.
        axis: The axis of rotation.
        limits: The limits of rotation in radians.
    """
    parent: Link
    child: Link
    transform: Transform
    axis: tuple[float, float, float]
    limits: tuple[float, float] | None

@dataclass
class SliderJoint:
    """
    Joint that allows translation along a single axis.

    Attributes:
        parent: The parent link of the joint.
        child: The child link of the joint.
        transform: The translation and rotation of the child link from the parent link.
        axis: The axis of translation.
        limits: The limits of translation in meters.
    """
    parent: Link
    child: Link
    transform: Transform
    axis: tuple[float, float, float]
    limits: tuple[float, float]

Joint = RigidJoint | SliderJoint | RevoluteJoint


class Model:
    """
    Defines a robot model.

    Attributes:
        name: The name of the model.
        joints: A list of joints that the model is composed of.
        base: The first link in the model kinematic chain.
        origin: An optional transform specifying the model translation and rotation.
    """
    def __init__(self, name: str, joints: list[Joint], base: Link, origin: Transform = Transform()):
        self.joints = joints
        self.base_link = base
        self.base_path = name + "/" + self.base_link.name
        self.link_path_map = {self.base_link.name: self.base_path}
        self.origin = origin
        self.pixi_root = Path(os.environ['PIXI_PROJECT_ROOT'])
        self.asset_path = self.pixi_root / "src" / "marionette" / "models" / name / "assets"
        self.parent_links_to_joints = {}

        for joint in self.joints:
            # Add joint to parent link's list of joints if it exists, otherwise create a new list
            self.parent_links_to_joints.setdefault(joint.parent.name, []).append(joint)

    def visualize(self):
        """Visualize the model in Rerun."""
        self._load_asset(self.base_path, self.base_link.visual, self.origin)  # Load base link
        self._traverse_joint_tree(self.base_path, self.base_link.name)

    def attach(self, other_model: "Model", joint_index: int, transform: Transform = Transform()):
        """Attach another model to this model at the specified joint and optional transform."""
        joint = self.joints[joint_index]
        other_model.origin = transform
        other_model.base_path = self.link_path_map[joint.child.name] + "/" + other_model.base_link.name
        self.link_path_map.update(other_model.link_path_map)
        other_model.visualize()

    def move_joint(self, joint_index: int, position: float):
        """Move the specified joint to the given position."""
        joint = self.joints[joint_index]
        if isinstance(joint, RigidJoint):
            return
        rotation = Rotation.from_euler('xyz', joint.transform.rotation()).as_matrix()
        rr_path = self.link_path_map[joint.child.name]
        axis_unit_vector = (rotation @ joint.axis) / np.linalg.norm(rotation @ joint.axis)
        joint_limits = joint.limits
        if joint_limits is not None:
            min_limit, max_limit = min(joint_limits), max(joint_limits)
            if min_limit > position or max_limit < position:
                return
        if isinstance(joint, SliderJoint):
            translation = axis_unit_vector * position + joint.transform.translation()
            rr.log(rr_path, rr.Transform3D(translation=translation, clear=False))
        elif isinstance(joint, RevoluteJoint):
            rotation = rr.RotationAxisAngle(axis=axis_unit_vector, radians=position)
            rr.log(rr_path, rr.Transform3D(rotation=rotation, clear=False))

    def copy(self, name: str, origin: Transform = Transform()) -> "Model":
        """Return a deep copy of the model with a new name and transform."""
        joints = deepcopy(self.joints)
        base_link = deepcopy(self.base_link)
        copied_model = Model(name=name, joints=joints, base=base_link, origin=origin)
        copied_model.link_path_map = deepcopy(self.link_path_map)
        copied_model.parent_links_to_joints = deepcopy(self.parent_links_to_joints)
        copied_model.asset_path = self.asset_path
        return copied_model

    def _traverse_joint_tree(self, rr_path: str, current_link_name: str):
        for joint in self.parent_links_to_joints.get(current_link_name, []):
            child_path = rr_path + "/" + joint.child.name
            self.link_path_map[joint.child.name] = child_path
            self._load_asset(child_path, joint.child.visual, joint.transform)
            self._traverse_joint_tree(child_path, joint.child.name)

    def _load_asset(self, rr_path: str, visual_file: str, transform: Transform = Transform()):
        rotation = Rotation.from_euler('xyz', transform.rotation()).as_matrix()
        translation = transform.translation()
        visual_path = f"{self.asset_path}/{visual_file}"
        rr.log(
            rr_path,
            rr.Asset3D(path=visual_path),
            rr.Transform3D(translation=translation, mat3x3=rotation)
        )
