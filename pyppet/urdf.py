import pyppet.format as pyf
import xml.etree.ElementTree as ET
from xml.dom import minidom


def geometry_to_urdf(geometry):
    if isinstance(geometry, pyf.Sphere):
        return ET.Element("sphere", radius=str(geometry.radius))
    elif isinstance(geometry, pyf.Box):
        return ET.Element("box", size=f"{geometry.length} {geometry.width} {geometry.height}")
    elif isinstance(geometry, pyf.Cylinder):
        return ET.Element("cylinder", radius=str(geometry.radius), length=str(geometry.height))
    elif isinstance(geometry, pyf.Mesh):
        return ET.Element("mesh", filename=geometry.filename,
            scale=f"{geometry.scale[0]} {geometry.scale[1]} {geometry.scale[2]}")
    return None


def pose_to_origin(pose: pyf.Pose):
    xyz = f"{pose.translation[0]} {pose.translation[1]} {pose.translation[2]}"
    rpy = f"{pose.rotation[0]} {pose.rotation[1]} {pose.rotation[2]}"
    return ET.Element("origin", xyz=xyz, rpy=rpy)


def visual_to_urdf(visual: pyf.Visual):
    visual_el = ET.Element("visual")

    if visual.geometry:
        geometry_el = ET.Element("geometry")
        shape = geometry_to_urdf(visual.geometry)
        if shape is not None:
            geometry_el.append(shape)
        visual_el.append(geometry_el)

    if visual.color:
        mat_el = ET.SubElement(visual_el, "material", name="custom")
        ET.SubElement(mat_el, "color", rgba=f"{visual.color[0]} {visual.color[1]} {visual.color[2]} 1.0")

    return visual_el


def inertial_to_urdf(physics: pyf.Physics):
    if not physics or physics.mass is None or physics.inertia is None:
        return None

    inertial_el = ET.Element("inertial")

    ET.SubElement(inertial_el, "mass", value=str(physics.mass))

    ixx, iyy, izz, ixy, ixz, iyz = physics.inertia
    ET.SubElement(
        inertial_el,
        "inertia",
        ixx=str(ixx),
        ixy=str(ixy),
        ixz=str(ixz),
        iyy=str(iyy),
        iyz=str(iyz),
        izz=str(izz)
    )

    if physics.center_of_mass:
        origin = pose_to_origin(physics.center_of_mass)
        inertial_el.append(origin)

    return inertial_el


def link_to_urdf(link: pyf.Link):
    link_el = ET.Element("link", name=link.name)

    if link.visual:
        visual_el = visual_to_urdf(link.visual)
        link_el.append(visual_el)

    if link.physics:
        inertial_el = inertial_to_urdf(link.physics)
        if inertial_el is not None:
            link_el.append(inertial_el)

    if link.collision:
        collision_el = ET.Element("collision")
        geometry_el = ET.Element("geometry")
        shape = geometry_to_urdf(link.collision)
        if shape is not None:
            geometry_el.append(shape)
            collision_el.append(geometry_el)
            link_el.append(collision_el)

    return link_el


def joint_to_urdf(joint: pyf.Joint):
    if isinstance(joint, pyf.RevoluteJoint):
        joint_type = "revolute"
    elif isinstance(joint, pyf.SliderJoint):
        joint_type = "prismatic"
    elif isinstance(joint, pyf.RigidJoint):
        joint_type = "fixed"

    joint_el = ET.Element("joint", name=f"{joint.parent.name}_to_{joint.child.name}", type=joint_type)
    ET.SubElement(joint_el, "parent", link=joint.parent.name)
    ET.SubElement(joint_el, "child", link=joint.child.name)

    origin_el = pose_to_origin(joint.pose)
    joint_el.append(origin_el)

    if isinstance(joint, pyf.MobileJoint):
        ET.SubElement(joint_el, "axis", xyz=f"{joint.axis[0]} {joint.axis[1]} {joint.axis[2]}")

        if joint.position_range:
            lower=str(joint.position_range[0])
            upper=str(joint.position_range[1])
        else:
            lower="0.0"
            upper="0.0"

        if joint.force_limit:
            effort = str(joint.force_limit)
        else:
            effort = "0.0"

        if joint.velocity_limit:
            velocity = str(joint.velocity_limit)
        else:
            velocity = "0.0"

        if joint.friction:
            friction = str(joint.friction)
        else:
            friction = "0.0"

        if joint.damping:
            damping = str(joint.damping)
        else:
            damping = "0.0"

        ET.SubElement(
            joint_el,
            "limit",
            lower=lower,
            upper=upper,
            velocity=velocity,
            effort=effort,
        )

        ET.SubElement(
            joint_el,
            "dynamics",
            friction=friction,
            damping=damping,
        )

    return joint_el


def pyppet_to_urdf(model: pyf.Model, file_name: str) -> str:
    """Convert a pyppet model object to a URDF string and save it to a file.

    Args:
        model (Model): The pyppet Model object to convert.
        file_name (str): The name of the file to save the URDF string to.

    Returns:
        str: The URDF string.
    """
    robot_el = ET.Element("robot", name=model.name)

    # Add links
    for link in model.links:
        robot_el.append(link_to_urdf(link))

    # Add joints
    for joint in model.joints:
        robot_el.append(joint_to_urdf(joint))

    # Convert to pretty XML string
    rough_urdf_string = ET.tostring(robot_el, 'utf-8')
    minidom_urdf = minidom.parseString(rough_urdf_string)
    pretty_urdf_string = minidom_urdf.toprettyxml(indent="  ")

    with open(file_name, 'w') as file:
        file.write(pretty_urdf_string)

    return pretty_urdf_string
