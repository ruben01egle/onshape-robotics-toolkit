"""
Xacro post-processing for URDF files produced by URDFSerializer.

This module reads back a URDF written by URDFSerializer().save(...) and writes a xacro
file that wraps every link and joint in a single ``<xacro:macro>`` and rewrites mesh paths
to ``package://`` form. There are two modes:

- **Arm** (:func:`convert_urdf_to_xacro`): macro ``<robot_name>_geometry``, optionally strips
  a legacy dummy root link, and appends a fixed ``flange`` link/joint at the tip of the
  kinematic chain. Tools are mounted on ``flange``.
- **Tool** (:func:`convert_tool_urdf_to_xacro`): macro ``tool`` with a single ``parent``
  param. All names are prefixed, meshes live under ``meshes/tools/<tool_name>/``, the tool's
  root link is attached to ``${parent}`` and a fixed ``tcp`` link/joint is appended.

Arm and tool files combine into one tree via ``<xacro:tool parent="flange"/>``.
"""

from __future__ import annotations

import shutil
from pathlib import Path

from loguru import logger
from lxml import etree as ET

from onshape_robotics_toolkit.mesh import SOURCE_MESH_DIR
from onshape_robotics_toolkit.models.link import _clean_origin_component
from onshape_robotics_toolkit.utilities.helpers import format_number

XACRO_NS = "http://www.ros.org/wiki/xacro"

FLANGE_LINK = "flange"
FLANGE_JOINT = "flange_joint"
TCP_LINK = "tcp"
TCP_JOINT = "tcp_joint"
TOOL_MACRO = "tool"
TOOL_PARENT_PARAM = "parent"
TOOLS_MESH_DIR = "tools"


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
            "Set tip_link explicitly to pick which tip the flange link/joint attaches to."
        )
    return next(iter(leaf_links))


def _validate_link(robot_elem: ET._Element, link_name: str, argument: str) -> None:
    link_names = sorted(name for link in robot_elem.findall("link") if (name := link.get("name")) is not None)
    if link_name not in link_names:
        raise ValueError(f"{argument}={link_name!r} is not a link of the URDF. Existing links: {link_names}")


def _format_triple(values: tuple[float, float, float]) -> str:
    """Format an origin xyz/rpy triple like URDFSerializer does (8 significant digits, noise snapped to 0)."""
    return " ".join(format_number(_clean_origin_component(float(v))) for v in values)


def _append_fixed_joint(
    macro: ET._Element,
    *,
    joint_name: str,
    parent_link: str,
    child_link: str,
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float],
) -> None:
    joint = ET.SubElement(macro, "joint", name=joint_name, type="fixed")
    ET.SubElement(joint, "origin", xyz=_format_triple(xyz), rpy=_format_triple(rpy))
    ET.SubElement(joint, "parent", link=parent_link)
    ET.SubElement(joint, "child", link=child_link)


def _write_xacro(xacro_root: ET._Element, xacro_path: str) -> None:
    ET.indent(xacro_root, space="    ")
    xacro_content = '<?xml version="1.0" ?>\n' + ET.tostring(xacro_root, pretty_print=True, encoding="unicode")
    Path(xacro_path).write_text(xacro_content, encoding="utf-8")


def convert_urdf_to_xacro(
    urdf_path: str,
    xacro_path: str,
    *,
    package_name: str,
    mesh_dir: str,
    strip_root_link: bool = False,
    tip_link: str | None = None,
    flange_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    flange_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    """
    Convert an arm URDF written by URDFSerializer().save(...) into a xacro macro file.

    The macro is named ``<robot_name>_geometry`` and ends with a fixed ``flange`` link
    (joint ``flange_joint``) on the tip link. No ``tcp`` is emitted; the tool files written
    by :func:`convert_tool_urdf_to_xacro` provide it.

    **Example:**
        >>> from onshape_robotics_toolkit.formats import convert_urdf_to_xacro
        >>> convert_urdf_to_xacro(
        ...     "robot.urdf",
        ...     "robot.urdf.xacro",
        ...     package_name="robot_description",
        ...     mesh_dir="meshes",
        ...     tip_link="tip_link_name",
        ...     flange_xyz=(0.0, 0.0, 0.148),
        ... )

    Args:
        urdf_path: Path to the source URDF (as written by URDFSerializer().save()).
        xacro_path: Path to write the resulting xacro file to.
        package_name: ROS package name used for `package://<package_name>/meshes/...` paths.
        mesh_dir: Directory the URDF's meshes were downloaded to (for strip_root_link cleanup).
        strip_root_link: Legacy escape hatch for a dummy fixed root part - strips the root
            link's visual/collision/inertial and deletes its mesh (if not shared). Leave
            False once the root link is a real part (e.g. via root_mate_name).
        tip_link: Name of the link to attach the flange joint to. If None, auto-detected as
            the sole leaf link (raises if the tree branches into more than one tip).
        flange_xyz: Position (meters) of the flange frame in the tip link's frame.
        flange_rpy: Orientation (radians, roll/pitch/yaw) of the flange frame in the tip link's frame.

    Raises:
        ValueError: If tip_link is not a link of the URDF, or cannot be auto-detected.
    """
    tree = ET.parse(urdf_path)  # noqa: S320
    source_root = tree.getroot()
    robot_name = source_root.get("name") or "robot"

    root_links, leaf_links = _find_root_and_leaf_links(source_root)
    root_link_name = next(iter(root_links)) if root_links else None

    resolved_tip_link = _resolve_tip_link(leaf_links, tip_link)
    _validate_link(source_root, resolved_tip_link, "tip_link")

    if strip_root_link and root_link_name is not None:
        _strip_root_link(source_root, root_link_name, mesh_dir)

    _rewrite_mesh_paths(source_root, package_name)

    xacro_root = ET.Element("robot", nsmap={"xacro": XACRO_NS})
    macro = ET.SubElement(xacro_root, f"{{{XACRO_NS}}}macro", name=f"{robot_name}_geometry")

    for element in list(source_root):
        macro.append(element)

    ET.SubElement(macro, "link", name=FLANGE_LINK)
    _append_fixed_joint(
        macro,
        joint_name=FLANGE_JOINT,
        parent_link=resolved_tip_link,
        child_link=FLANGE_LINK,
        xyz=flange_xyz,
        rpy=flange_rpy,
    )

    _write_xacro(xacro_root, xacro_path)


def _prefix_names(robot_elem: ET._Element, prefix: str) -> None:
    """Prefix every link/joint/visual/collision/material name and every reference to a link or joint."""

    def prefix_attr(element: ET._Element | None, attr: str) -> None:
        if element is not None and (value := element.get(attr)) is not None:
            element.set(attr, prefix + value)

    for link in robot_elem.findall("link"):
        prefix_attr(link, "name")
    for joint in robot_elem.findall("joint"):
        prefix_attr(joint, "name")
        prefix_attr(joint.find("parent"), "link")
        prefix_attr(joint.find("child"), "link")
        prefix_attr(joint.find("mimic"), "joint")
    for tag in ("visual", "collision", "material"):
        for element in robot_elem.iter(tag):
            prefix_attr(element, "name")


def _relocate_tool_meshes(robot_elem: ET._Element, mesh_dir: str, tool_name: str) -> None:
    """Point every mesh at ``meshes/tools/<tool_name>/<path in mesh_dir>`` and move the files on
    disk (and their full-resolution ``source/`` copies) to ``<mesh_dir>/tools/<tool_name>/``,
    so the contents of mesh_dir map 1:1 onto the package's ``meshes/`` directory.
    """
    mesh_root = Path(mesh_dir)
    tool_mesh_dir = mesh_root / TOOLS_MESH_DIR / tool_name

    relative_paths = set()
    for mesh in robot_elem.iter("mesh"):
        filename = mesh.get("filename")
        if not filename or filename.startswith("package://"):
            continue
        relative_path = _mesh_path_in_dir(filename, mesh_dir)
        relative_paths.add(relative_path)
        mesh.set("filename", f"meshes/{TOOLS_MESH_DIR}/{tool_name}/{relative_path}")

    for relative_path in sorted(relative_paths):
        moves = [
            (mesh_root / relative_path, tool_mesh_dir / relative_path, True),
            (mesh_root / SOURCE_MESH_DIR / relative_path, tool_mesh_dir / SOURCE_MESH_DIR / relative_path, False),
        ]
        for src, dst, required in moves:
            if src.exists():
                dst.parent.mkdir(parents=True, exist_ok=True)
                shutil.move(str(src), str(dst))
            elif required and not dst.exists():
                logger.warning(f"Mesh {relative_path} not found in {mesh_dir} or {tool_mesh_dir}")


def convert_tool_urdf_to_xacro(
    urdf_path: str,
    xacro_path: str,
    *,
    package_name: str,
    mesh_dir: str,
    tool_name: str,
    link_prefix: str | None = None,
    attach_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    attach_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    tcp_parent_link: str | None = None,
    tcp_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    tcp_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    """
    Convert an end-effector tool URDF written by URDFSerializer().save(...) into a xacro macro file.

    The source URDF comes from an Onshape assembly that contains only the tool, its root link
    being the part that mounts on the arm's flange. The macro is always
    ``<xacro:macro name="tool" params="parent">``, so every tool file has the same interface
    and is used as ``<xacro:tool parent="flange"/>``. It contains:

    - all tool links/joints, every name prefixed with link_prefix (including parent/child and
      ``<mimic joint>`` references and ``<visual>``/``<collision>``/``<material>`` names);
      inertials are copied unchanged,
    - a fixed ``<link_prefix>attach_joint`` from ``${parent}`` to the tool's root link,
    - an unprefixed ``tcp`` link with a fixed, unprefixed ``tcp_joint`` on tcp_parent_link.

    Mesh paths become ``package://<package_name>/meshes/tools/<tool_name>/<file>`` (keeping
    subdirectories such as ``collision/``), and the files are moved on disk to
    ``<mesh_dir>/tools/<tool_name>/``, so the contents of mesh_dir can be copied 1:1 into the
    package's ``meshes/`` directory.

    **Example:**
        >>> from onshape_robotics_toolkit.formats import convert_tool_urdf_to_xacro
        >>> convert_tool_urdf_to_xacro(
        ...     "output/gripper_v1.urdf",
        ...     "output/gripper_v1.urdf.xacro",
        ...     package_name="robotarm_description",
        ...     mesh_dir="output/meshes",
        ...     tool_name="gripper_v1",
        ...     tcp_xyz=(0.0, 0.0, 0.12),
        ... )

    Args:
        urdf_path: Path to the source tool URDF (as written by URDFSerializer().save()).
        xacro_path: Path to write the resulting xacro file to.
        package_name: ROS package name used for ``package://`` mesh paths.
        mesh_dir: Directory the URDF's meshes were downloaded to.
        tool_name: Tool name, e.g. ``"gripper_v1"``; used for the default prefix and the mesh subdirectory.
        link_prefix: Prefix for all link/joint names. Defaults to ``f"{tool_name}_"``.
        attach_xyz: Position (meters) of the tool's root link in the parent (flange) frame.
        attach_rpy: Orientation (radians) of the tool's root link in the parent (flange) frame.
        tcp_parent_link: Unprefixed name (as in the source URDF) of the link the tcp attaches to.
            Defaults to the tool's root link.
        tcp_xyz: Position (meters) of the tcp frame in tcp_parent_link's frame.
        tcp_rpy: Orientation (radians) of the tcp frame in tcp_parent_link's frame.

    Raises:
        ValueError: If the URDF does not have exactly one root link, or tcp_parent_link is not a link of it.
    """
    prefix = f"{tool_name}_" if link_prefix is None else link_prefix

    tree = ET.parse(urdf_path)  # noqa: S320
    source_root = tree.getroot()

    root_links, _ = _find_root_and_leaf_links(source_root)
    if len(root_links) != 1:
        raise ValueError(
            f"Tool URDF {urdf_path} has {len(root_links)} root links ({sorted(root_links)}) instead of exactly one."
        )
    root_link_name = next(iter(root_links))

    resolved_tcp_parent = root_link_name if tcp_parent_link is None else tcp_parent_link
    _validate_link(source_root, resolved_tcp_parent, "tcp_parent_link")

    movable_joints = sorted(
        str(joint.get("name")) for joint in source_root.findall("joint") if joint.get("type") != "fixed"
    )
    if movable_joints:
        logger.warning(
            f"Tool {tool_name} has non-fixed joints {movable_joints}; they are kept, "
            "but the consumer currently only supports rigid tools."
        )

    _prefix_names(source_root, prefix)
    _relocate_tool_meshes(source_root, mesh_dir, tool_name)
    _rewrite_mesh_paths(source_root, package_name)

    xacro_root = ET.Element("robot", nsmap={"xacro": XACRO_NS})
    macro = ET.SubElement(xacro_root, f"{{{XACRO_NS}}}macro", name=TOOL_MACRO, params=TOOL_PARENT_PARAM)

    for element in list(source_root):
        macro.append(element)

    _append_fixed_joint(
        macro,
        joint_name=f"{prefix}attach_joint",
        parent_link=f"${{{TOOL_PARENT_PARAM}}}",
        child_link=prefix + root_link_name,
        xyz=attach_xyz,
        rpy=attach_rpy,
    )
    ET.SubElement(macro, "link", name=TCP_LINK)
    _append_fixed_joint(
        macro,
        joint_name=TCP_JOINT,
        parent_link=prefix + resolved_tcp_parent,
        child_link=TCP_LINK,
        xyz=tcp_xyz,
        rpy=tcp_rpy,
    )

    _write_xacro(xacro_root, xacro_path)
