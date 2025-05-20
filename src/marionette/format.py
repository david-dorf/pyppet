import trimesh.visual
import rerun as rr
from dataclasses import dataclass
from scipy.spatial.transform import Rotation
import numpy as np
import os
from pathlib import Path


@dataclass
class Link:
    """
    Rigid component in a robot. Contains a mesh file, a name, and an optional mass.
    Base links serve as the foundation of the robot. There should only be one base link per robot.
    """
    file: str
    name: str
    is_base: bool = False
    mass: float | None = None

@dataclass
class Transform:
    """Change in position and orientation of a rigid body."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    def translation(self): return self.x, self.y, self.z
    def rotation(self): return self.roll, self.pitch, self.yaw

@dataclass
class JointBase:
    """Base class for all joints."""
    parent: Link
    child: Link
    transform: Transform

@dataclass
class RevoluteJoint(JointBase):
    """
    Joint that allows rotation around a single axis.
    Limits are optional and the joint will rotate continuously without limits.

    Attributes:
        axis: The axis of rotation.
        limits: The limits of rotation in radians.
    """
    axis: tuple[float, float, float]
    limits: tuple[float, float] | None

@dataclass
class SliderJoint(JointBase):
    """
    Joint that allows translation along a single axis.

    Attributes:
        axis: The axis of translation.
        limits: The limits of translation in meters.
    """
    axis: tuple[float, float, float]
    limits: tuple[float, float]

@dataclass
class RigidJoint(JointBase):
    """Joint that does not allow translation or rotation."""
    pass

Joint = RigidJoint | SliderJoint | RevoluteJoint


class Model:
    def __init__(self, name: str, joints: dict[str, Joint], transform: Transform | None = None):
        self.transform = transform if transform is not None else Transform()
        self.pixi_root = Path(os.environ['PIXI_PROJECT_ROOT'])
        self.mesh_path = self.pixi_root / "src" / "marionette" / "models" / name / "meshes"
        self.joints = joints
        self.base_link = self._get_base_link()
        self.base_path = name + "/" + self.base_link.name
        self.link_path_map = {self.base_link.name: self.base_path}
        self.parent_link_to_joints = {}  # Mapping of links to joints with them as the parent
        for joint_name, joint in self.joints.items():
            # Create an empty list for each parent link, and then fill it with joints
            self.parent_link_to_joints.setdefault(joint.parent.name, []).append(joint)

    def visualize(self):
        """Visualize the model in Rerun."""
        self._load_mesh(self.base_path, self.base_link.file, self.transform)
        self._traverse_joint_tree(self.base_path, self.base_link.name)

    def attach(self, other_model: "Model", joint_name: str, transform: Transform | None = Transform()):
        """Attach another model to this model at the specified joint and optional transform."""
        joint = self.joints[joint_name]
        other_model.transform = transform
        other_model.base_path = self.link_path_map[joint.child.name] + "/" + other_model.base_link.name
        self.link_path_map.update(other_model.link_path_map)
        other_model.visualize()

    def move_joint(self, joint_name: str, position: float):
        """Move the specified joint to the given position."""
        joint = self.joints[joint_name]
        if isinstance(joint, RigidJoint):
            raise ValueError("Rigid joints cannot be moved")
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

    def _get_base_link(self) -> Link:
        base_link = None
        for joint in self.joints.values():
            if joint.parent.is_base:
                if base_link is not None:
                    raise ValueError("Model has multiple base links")
                base_link = joint.parent
        if base_link is None:
            raise ValueError("Model has no base link")
        return base_link

    def _traverse_joint_tree(self, rr_path: str, current_link_name: str):
        for joint in self.parent_link_to_joints.get(current_link_name, []):
            child_path = rr_path + "/" + joint.child.name
            self.link_path_map[joint.child.name] = child_path
            self._load_mesh(child_path, joint.child.file, joint.transform)
            self._traverse_joint_tree(child_path, joint.child.name)

    def _load_mesh(self, rr_path: str, file: str, transform: Transform):
        mesh = trimesh.load_mesh(f"{self.mesh_path}/{file}")
        visual = mesh.visual
        if (texture := getattr(visual, 'uv', None)) is not None:
            texture[:, 1] = 1.0 - texture[:, 1]
        mat = getattr(visual, 'material', None)
        albedo = getattr(mat, 'baseColorTexture', None)
        colors = getattr(mat, 'baseColorFactor', None)
        rotation = Rotation.from_euler('xyz', transform.rotation()).as_matrix()

        rr.log(
            rr_path,
            rr.Mesh3D(
                vertex_positions = mesh.vertices,
                triangle_indices = mesh.faces,
                vertex_normals = mesh.vertex_normals,
                vertex_colors = colors,
                vertex_texcoords = texture,
                albedo_texture = albedo
            ),
            rr.Transform3D(translation=transform.translation(), mat3x3=rotation)
        )
