import rerun as rr
import pyppet.format as pf
from scipy.spatial.transform import Rotation as R


class RerunVisualizer:
    def __init__(self, model: pf.Model):
        self.model = model
        self.current_asset_hierarchy = ""
        rr.init("pyppet_rerun_example", spawn=True)

    def visualize(self):
        for joint in self.model.joints:
            self._get_asset_hierarchy(joint)
            self._log_geometry(joint)
            self._log_transform(joint)

    def _get_asset_hierarchy(self, joint: pf.Joint):
        """Returns a string representing the asset hierarchy of the given link."""
        asset_hierarchy = ""
        for traversed_joint in self.model.traverse_joint_tree(joint):
            asset_hierarchy = f"/{traversed_joint.child.name}" + asset_hierarchy
        asset_hierarchy = self.model.name + "/" + self.model.base_link.name + asset_hierarchy
        self.current_asset_hierarchy = asset_hierarchy

    def _log_geometry(self, joint: pf.Joint):
        if joint.child.visual is not None:
            geometry = joint.child.visual.geometry
            color = joint.child.visual.color
            if isinstance(geometry, pf.Box):
                box_dimensions = [geometry.length, geometry.width, geometry.height]
                rr.log(
                    self.current_asset_hierarchy,
                    rr.Boxes3D(
                        half_sizes=[box_dimensions[0]/2, box_dimensions[1]/2, box_dimensions[2]/2],
                        colors=color,
                        fill_mode = "solid"
                    )
                )
            elif isinstance(geometry, pf.Sphere):
                sphere_radius = geometry.radius
                rr.log(
                    self.current_asset_hierarchy,
                    rr.Ellipsoids3D(
                        half_sizes=[[sphere_radius, sphere_radius, sphere_radius]],
                        colors=color,
                        fill_mode = "solid"
                    )
                )
            elif isinstance(geometry, pf.Cylinder):
                cylinder_radius = geometry.radius
                cylinder_height = geometry.height
                rr.log(
                    self.current_asset_hierarchy,
                    rr.Cylinders3D(
                        radii=[cylinder_radius],
                        lengths=[cylinder_height],
                        colors=color,
                        fill_mode = "solid"
                    ),
                )
            elif isinstance(geometry, pf.Mesh):
                rr.log(
                    self.current_asset_hierarchy,
                    rr.Asset3D(
                        path=geometry.filename,
                    )
                )
            else:
                raise ValueError(f"Unsupported geometry type: {type(geometry)}")

    def _log_transform(self, joint: pf.Joint):
        if joint.pose is not None:
            rr.log(
                self.current_asset_hierarchy,
                rr.Transform3D(
                    translation=joint.pose.translation,
                    mat3x3=R.from_euler('xyz', joint.pose.rotation).as_matrix(),
                    # TODO: Scale always operates from origin rather than center, needs fix
                )
            )
