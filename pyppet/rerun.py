import rerun as rr
import pyppet.format as pf


def visualize_rerun(model: pf.Model):
    rr.init("", spawn=True)
    for joint in model.joints:
        # Obtain hierarchy of links in model joint tree
        asset_hierarchy = ""
        for current_joint in model.traverse_joint_tree(joint):
            asset_hierarchy = f"/{current_joint.child.name}" + asset_hierarchy
        asset_hierarchy = model.name + "/" + model.base_link.name + asset_hierarchy

        # Log visual elements to Rerun
        scale = None
        if joint.child.visual is not None:
            geometry = joint.child.visual.geometry
            color = joint.child.visual.color
            if isinstance(geometry, pf.Box):
                box_dimensions = [geometry.length, geometry.width, geometry.height]
                rr.log(
                    asset_hierarchy,
                    rr.Boxes3D(
                        half_sizes=[box_dimensions],
                        colors=color,
                        fill_mode = "solid"
                    )
                )
            elif isinstance(geometry, pf.Sphere):
                sphere_radius = geometry.radius
                rr.log(
                    asset_hierarchy,
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
                    asset_hierarchy,
                    rr.Cylinders3D(
                        radii=[cylinder_radius],
                        lengths=[cylinder_height],
                        colors=color,
                        fill_mode = "solid"
                    ),
                )
            elif isinstance(geometry, pf.Mesh):
                scale = geometry.scale
                rr.log(
                    asset_hierarchy,
                    rr.Asset3D(
                        path=geometry.filename,
                    )
                )
                rr.log(
                    asset_hierarchy,
                    rr.Transform3D(
                        scale=scale,
                    )
                )
            else:
                raise ValueError(f"Unsupported geometry type: {type(geometry)}")

        # Log joint pose as 3D transformations in Rerun
        if joint.pose is not None:
            rr.log(
                asset_hierarchy,
                rr.Transform3D(
                    translation=joint.pose.translation,
                    scale = scale if scale is not None else [1, 1, 1]
                )
            )
