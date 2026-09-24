"""
Xacro post-processing for URDF files produced by URDFSerializer.

This module reads back a URDF written by URDFSerializer().save(...) and writes a
sibling ``<urdf_path>.xacro`` that wraps every link and joint in a single
``<xacro:macro>``, rewrites mesh paths to ``package://`` form, optionally strips a
legacy dummy root link, and appends a fixed ``tcp`` link/joint at the tip of the
kinematic chain.
"""

from __future__ import annotations

from pathlib import Path

from lxml import etree as ET

from onshape_robotics_toolkit.mesh import SOURCE_MESH_DIR

XACRO_NS = "http://www.ros.org/wiki/xacro"


def _find_root_and_leaf_links(robot_elem: ET._Element) -> tuple[set[str], set[str]]:
    """Return (root link names, leaf link names) from a <robot> element's <joint>s.

    A root link is never a <child>; a leaf link is never a <parent>.
    """
    all_links = {link.get("name") for link in robot_elem.findall("link")}
    parents = set()
    children = set()
    for joint in robot_elem.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is not None and parent.get("link") is not None:
            parents.add(parent.get("link"))
        if child is not None and child.get("link") is not None:
            children.add(child.get("link"))

    root_links = all_links - children
    leaf_links = all_links - parents
    return root_links, leaf_links


def _rewrite_mesh_paths(robot_elem: ET._Element, package_name: str) -> None:
    for mesh in robot_elem.iter("mesh"):
        filename = mesh.get("filename")
        if not filename or filename.startswith("package://"):
            continue
        mesh.set("filename", f"package://{package_name}/{filename}")


def _mesh_path_in_dir(filename: str, mesh_dir: str) -> str:
    """Return a URDF mesh filename relative to mesh_dir, keeping subdirectories such as
    ``collision/`` (e.g. ``meshes/collision/a.stl`` with mesh_dir ``meshes`` -> ``collision/a.stl``).
    """
    parts = filename.replace("\\", "/").split("/")
    mesh_dir_name = Path(mesh_dir).name
    if mesh_dir_name in parts[:-1]:
        index = len(parts) - 1 - parts[::-1].index(mesh_dir_name)
        return "/".join(parts[index + 1 :])
    return parts[-1]


def _strip_root_link(
    robot_elem: ET._Element,
    root_link_name: str,
    mesh_dir: str,
) -> None:
    """Reduce the root link to a bare <link name="..."/>, deleting its mesh files on
    disk (and the full-resolution copy in ``source/``, if any) unless another link in
    this URDF still references the same mesh file.
    """
    # A link's <visual> and <collision> may reference the same mesh, so count
    # distinct *links* referencing a given file, not raw <mesh> element occurrences.
    links_by_mesh_path: dict[str, set[str]] = {}
    for link in robot_elem.findall("link"):
        link_name = link.get("name")
        for mesh in link.iter("mesh"):
            filename = mesh.get("filename")
            if filename:
                links_by_mesh_path.setdefault(_mesh_path_in_dir(filename, mesh_dir), set()).add(link_name)

    root_link = next(
        (link for link in robot_elem.findall("link") if link.get("name") == root_link_name),
        None,
    )
    if root_link is None:
        return

    root_mesh_paths = {
        _mesh_path_in_dir(filename, mesh_dir)
        for mesh in root_link.iter("mesh")
        if (filename := mesh.get("filename")) is not None
    }

    for child in list(root_link):
        root_link.remove(child)

    for relative_path in root_mesh_paths:
        if len(links_by_mesh_path.get(relative_path, set())) > 1:
            continue
        for mesh_path in (Path(mesh_dir) / relative_path, Path(mesh_dir) / SOURCE_MESH_DIR / relative_path):
            if mesh_path.exists():
                mesh_path.unlink()


def _resolve_tip_link(leaf_links: set[str], tip_link: str | None) -> str:
    if tip_link is not None:
        return tip_link

    if len(leaf_links) != 1:
        raise ValueError(
            "tip_link was not set and the kinematic tree has "
            f"{len(leaf_links)} leaf links ({sorted(leaf_links)}) instead of exactly one. "
            "Set tip_link explicitly to pick which tip the tcp link/joint attaches to."
        )
    return next(iter(leaf_links))


def convert_urdf_to_xacro(
    urdf_path: str,
    xacro_path: str,
    *,
    package_name: str,
    mesh_dir: str,
    strip_root_link: bool = False,
    tip_link: str | None = None,
    tcp_offset_z: float = 0.0,
) -> None:
    """
    Convert a URDF file written by URDFSerializer().save(...) into a xacro macro file.

    **Example:**
        >>> from onshape_robotics_toolkit.formats import convert_urdf_to_xacro
        >>> convert_urdf_to_xacro(
        ...     "robot.urdf",
        ...     "robot.urdf.xacro",
        ...     package_name="robot_description",
        ...     mesh_dir="meshes",
        ...     tip_link="tip_link_name",
        ...     tcp_offset_z=0.148,
        ... )

    Args:
        urdf_path: Path to the source URDF (as written by URDFSerializer().save()).
        xacro_path: Path to write the resulting xacro file to.
        package_name: ROS package name used for `package://<package_name>/meshes/...` paths.
        mesh_dir: Directory the URDF's meshes were downloaded to (for strip_root_link cleanup).
        strip_root_link: Legacy escape hatch for a dummy fixed root part - strips the root
            link's visual/collision/inertial and deletes its mesh (if not shared). Leave
            False once the root link is a real part (e.g. via root_mate_name).
        tip_link: Name of the link to attach the tcp joint to. If None, auto-detected as
            the sole leaf link (raises if the tree branches into more than one tip).
        tcp_offset_z: Z offset (meters) of the tcp frame from the tip link's own frame.
    """
    tree = ET.parse(urdf_path)
    source_root = tree.getroot()
    robot_name = source_root.get("name") or "robot"

    root_links, leaf_links = _find_root_and_leaf_links(source_root)
    root_link_name = next(iter(root_links)) if root_links else None

    if strip_root_link and root_link_name is not None:
        _strip_root_link(source_root, root_link_name, mesh_dir)

    _rewrite_mesh_paths(source_root, package_name)

    resolved_tip_link = _resolve_tip_link(leaf_links, tip_link)

    xacro_root = ET.Element("robot", nsmap={"xacro": XACRO_NS})
    macro = ET.SubElement(xacro_root, f"{{{XACRO_NS}}}macro", name=f"{robot_name}_geometry")

    for element in list(source_root):
        macro.append(element)

    ET.SubElement(macro, "link", name="tcp")
    tcp_joint = ET.SubElement(macro, "joint", name="tcp_joint", type="fixed")
    ET.SubElement(tcp_joint, "origin", xyz=f"0 0 {tcp_offset_z}", rpy="0 0 0")
    ET.SubElement(tcp_joint, "parent", link=resolved_tip_link)
    ET.SubElement(tcp_joint, "child", link="tcp")

    ET.indent(xacro_root, space="    ")

    xacro_content = '<?xml version="1.0" ?>\n' + ET.tostring(xacro_root, pretty_print=True, encoding="unicode")
    Path(xacro_path).write_text(xacro_content, encoding="utf-8")
