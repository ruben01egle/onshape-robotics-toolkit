"""Tests for the arm / tool xacro post-processing (no Onshape API calls)."""

from __future__ import annotations

from pathlib import Path

import pytest
from loguru import logger
from lxml import etree as ET

from onshape_robotics_toolkit.formats import convert_tool_urdf_to_xacro, convert_urdf_to_xacro
from onshape_robotics_toolkit.formats.xacro import XACRO_NS

PACKAGE = "robotarm_description"

ARM_URDF = """<?xml version="1.0" ?>
<robot name="robotarm">
  <link name="Base_1">
    <visual name="Base_1-visual">
      <geometry><mesh filename="meshes/Base_1.stl"/></geometry>
      <material name="Base_1-material"><color rgba="0.5 0.5 0.5 1"/></material>
    </visual>
    <collision name="Base_1-collision">
      <geometry><mesh filename="meshes/collision/Base_1_collision_0.stl"/></geometry>
    </collision>
  </link>
  <link name="Stage6_1">
    <inertial>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <mass value="0.8"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0005"/>
    </inertial>
  </link>
  <joint name="joint_1" type="revolute">
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <parent link="Base_1"/>
    <child link="Stage6_1"/>
    <axis xyz="0 0 1"/>
    <limit effort="1" velocity="1" lower="-1" upper="1"/>
  </joint>
</robot>
"""

TOOL_URDF = """<?xml version="1.0" ?>
<robot name="gripper">
  <link name="Base_1">
    <visual name="Base_1-visual">
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="meshes/Base_1.stl"/></geometry>
      <material name="Base_1-material"><color rgba="0.5 0.5 0.5 1"/></material>
    </visual>
    <collision name="Base_1-collision">
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="meshes/collision/Base_1_collision_0.stl"/></geometry>
    </collision>
    <inertial>
      <origin xyz="0.001 0 0.02" rpy="0 0 0"/>
      <mass value="0.25"/>
      <inertia ixx="1.2345678e-05" ixy="0" ixz="0" iyy="2e-05" iyz="0" izz="3e-05"/>
    </inertial>
  </link>
  <link name="Jaw_1">
    <visual name="Jaw_1-visual">
      <geometry><mesh filename="meshes/Jaw_1.stl"/></geometry>
      <material name="Jaw_1-material"><color rgba="0.5 0.5 0.5 1"/></material>
    </visual>
  </link>
  <link name="Jaw_2"/>
  <joint name="jaw_1_joint" type="fixed">
    <origin xyz="0 0.01 0.05" rpy="0 0 0"/>
    <parent link="Base_1"/>
    <child link="Jaw_1"/>
  </joint>
  <joint name="jaw_2_joint" type="fixed">
    <origin xyz="0 -0.01 0.05" rpy="0 0 0"/>
    <parent link="Base_1"/>
    <child link="Jaw_2"/>
  </joint>
</robot>
"""

TOOL_MESHES = ["Base_1.stl", "collision/Base_1_collision_0.stl", "Jaw_1.stl", "source/Base_1.stl"]


def _write(path: Path, content: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")
    return path


def _macro(xacro_path: Path) -> ET._Element:
    root = ET.parse(str(xacro_path)).getroot()  # noqa: S320
    macros = root.findall(f"{{{XACRO_NS}}}macro")
    assert len(macros) == 1
    return macros[0]


def _names(elem: ET._Element, tag: str) -> list[str]:
    return [e.get("name") for e in elem.findall(tag)]


@pytest.fixture
def arm_xacro(tmp_path: Path) -> Path:
    urdf = _write(tmp_path / "arm" / "robotarm.urdf", ARM_URDF)
    xacro_path = tmp_path / "arm" / "robotarm.urdf.xacro"
    convert_urdf_to_xacro(
        str(urdf),
        str(xacro_path),
        package_name=PACKAGE,
        mesh_dir=str(tmp_path / "arm" / "meshes"),
        flange_xyz=(0.0, 0.0, 0.148),
    )
    return xacro_path


@pytest.fixture
def tool_dir(tmp_path: Path) -> Path:
    out = tmp_path / "tool"
    _write(out / "gripper_v1.urdf", TOOL_URDF)
    for mesh in TOOL_MESHES:
        _write(out / "meshes" / mesh, mesh)
    return out


def _convert_tool(tool_dir: Path, **kwargs) -> Path:
    xacro_path = tool_dir / "gripper_v1.urdf.xacro"
    convert_tool_urdf_to_xacro(
        str(tool_dir / "gripper_v1.urdf"),
        str(xacro_path),
        package_name=PACKAGE,
        mesh_dir=str(tool_dir / "meshes"),
        tool_name="gripper_v1",
        **kwargs,
    )
    return xacro_path


# --- arm mode ---------------------------------------------------------------------------------


def test_arm_appends_flange_and_no_tcp(arm_xacro: Path) -> None:
    macro = _macro(arm_xacro)
    assert macro.get("name") == "robotarm_geometry"
    assert _names(macro, "link") == ["Base_1", "Stage6_1", "flange"]
    assert _names(macro, "joint") == ["joint_1", "flange_joint"]

    flange_joint = macro.findall("joint")[-1]
    assert flange_joint.get("type") == "fixed"
    assert flange_joint.find("origin").attrib == {"xyz": "0 0 0.148", "rpy": "0 0 0"}
    assert flange_joint.find("parent").get("link") == "Stage6_1"
    assert flange_joint.find("child").get("link") == "flange"
    assert "tcp" not in arm_xacro.read_text()


def test_arm_keeps_mesh_rewrite_and_limits(arm_xacro: Path) -> None:
    macro = _macro(arm_xacro)
    filenames = [m.get("filename") for m in macro.iter("mesh")]
    assert filenames == [
        f"package://{PACKAGE}/meshes/Base_1.stl",
        f"package://{PACKAGE}/meshes/collision/Base_1_collision_0.stl",
    ]
    assert macro.find("joint/limit").attrib == {"effort": "1", "velocity": "1", "lower": "-1", "upper": "1"}


def test_arm_flange_origin_formatting(tmp_path: Path) -> None:
    urdf = _write(tmp_path / "robotarm.urdf", ARM_URDF)
    xacro_path = tmp_path / "robotarm.urdf.xacro"
    convert_urdf_to_xacro(
        str(urdf),
        str(xacro_path),
        package_name=PACKAGE,
        mesh_dir=str(tmp_path / "meshes"),
        tip_link="Stage6_1",
        flange_xyz=(0.1 + 0.2, -1e-12, 1.0),
        flange_rpy=(3.141592653589793, 0, -1.5707963267948966),
    )
    origin = _macro(xacro_path).find("joint[@name='flange_joint']/origin")
    assert origin.attrib == {"xyz": "0.3 0 1", "rpy": "3.1415927 0 -1.5707963"}


def test_arm_invalid_tip_link_raises(tmp_path: Path) -> None:
    urdf = _write(tmp_path / "robotarm.urdf", ARM_URDF)
    xacro_path = tmp_path / "robotarm.urdf.xacro"
    with pytest.raises(ValueError, match=r"tip_link='Stage7_1'.*\['Base_1', 'Stage6_1'\]"):
        convert_urdf_to_xacro(
            str(urdf), str(xacro_path), package_name=PACKAGE, mesh_dir=str(tmp_path / "meshes"), tip_link="Stage7_1"
        )
    assert not xacro_path.exists()


# --- tool mode --------------------------------------------------------------------------------


def test_tool_macro_interface_and_names(tool_dir: Path) -> None:
    macro = _macro(_convert_tool(tool_dir, attach_xyz=(0, 0, 0.002), tcp_xyz=(0, 0, 0.12)))
    assert macro.get("name") == "tool"
    assert macro.get("params") == "parent"

    assert _names(macro, "link") == ["gripper_v1_Base_1", "gripper_v1_Jaw_1", "gripper_v1_Jaw_2", "tcp"]
    assert _names(macro, "joint") == [
        "gripper_v1_jaw_1_joint",
        "gripper_v1_jaw_2_joint",
        "gripper_v1_attach_joint",
        "tcp_joint",
    ]
    for tag in ("visual", "collision", "material"):
        assert all(name.startswith("gripper_v1_") for name in (e.get("name") for e in macro.iter(tag)))

    attach = macro.find("joint[@name='gripper_v1_attach_joint']")
    assert attach.get("type") == "fixed"
    assert attach.find("origin").attrib == {"xyz": "0 0 0.002", "rpy": "0 0 0"}
    assert attach.find("parent").get("link") == "${parent}"
    assert attach.find("child").get("link") == "gripper_v1_Base_1"

    tcp = macro.find("joint[@name='tcp_joint']")
    assert tcp.get("type") == "fixed"
    assert tcp.find("origin").attrib == {"xyz": "0 0 0.12", "rpy": "0 0 0"}
    assert tcp.find("parent").get("link") == "gripper_v1_Base_1"
    assert tcp.find("child").get("link") == "tcp"


def test_tool_references_consistent(tool_dir: Path) -> None:
    macro = _macro(_convert_tool(tool_dir, tcp_parent_link="Jaw_1"))
    links = set(_names(macro, "link"))
    for joint in macro.findall("joint"):
        parent = joint.find("parent").get("link")
        assert parent == "${parent}" or parent in links
        assert joint.find("child").get("link") in links
    assert macro.find("joint[@name='tcp_joint']/parent").get("link") == "gripper_v1_Jaw_1"


def test_tool_custom_prefix(tool_dir: Path) -> None:
    macro = _macro(_convert_tool(tool_dir, link_prefix="g_"))
    assert _names(macro, "link") == ["g_Base_1", "g_Jaw_1", "g_Jaw_2", "tcp"]
    assert "g_attach_joint" in _names(macro, "joint")
    assert macro.find("joint[@name='tcp_joint']/parent").get("link") == "g_Base_1"


def test_tool_mesh_paths_and_files(tool_dir: Path) -> None:
    macro = _macro(_convert_tool(tool_dir))
    base = f"package://{PACKAGE}/meshes/tools/gripper_v1"
    assert [m.get("filename") for m in macro.iter("mesh")] == [
        f"{base}/Base_1.stl",
        f"{base}/collision/Base_1_collision_0.stl",
        f"{base}/Jaw_1.stl",
    ]

    # mesh_dir maps 1:1 onto the package's meshes/ directory
    mesh_dir = tool_dir / "meshes"
    on_disk = sorted(str(p.relative_to(mesh_dir)) for p in mesh_dir.rglob("*") if p.is_file())
    assert on_disk == sorted(f"tools/gripper_v1/{m}" for m in TOOL_MESHES)
    assert (mesh_dir / "tools/gripper_v1/collision/Base_1_collision_0.stl").read_text() == (
        "collision/Base_1_collision_0.stl"
    )

    # Re-running on the same source URDF keeps working (files already moved)
    macro = _macro(_convert_tool(tool_dir))
    assert macro.find("link/visual/geometry/mesh").get("filename") == f"{base}/Base_1.stl"


def test_tool_inertials_unchanged(tool_dir: Path) -> None:
    source = ET.fromstring(TOOL_URDF.encode()).find("link[@name='Base_1']/inertial")  # noqa: S320
    macro = _macro(_convert_tool(tool_dir))
    inertials = list(macro.iter("inertial"))
    assert len(inertials) == 1
    assert [(e.tag, dict(e.attrib)) for e in inertials[0].iter()] == [(e.tag, dict(e.attrib)) for e in source.iter()]


def test_tool_invalid_tcp_parent_raises(tool_dir: Path) -> None:
    with pytest.raises(ValueError, match=r"tcp_parent_link='Finger_9'.*\['Base_1', 'Jaw_1', 'Jaw_2'\]"):
        _convert_tool(tool_dir, tcp_parent_link="Finger_9")


def test_tool_multiple_roots_raises(tool_dir: Path) -> None:
    urdf = tool_dir / "gripper_v1.urdf"
    urdf.write_text(TOOL_URDF.replace("</robot>", '  <link name="Loose_1"/>\n</robot>'), encoding="utf-8")
    with pytest.raises(ValueError, match="2 root links"):
        _convert_tool(tool_dir)


def test_tool_movable_joints_kept_with_warning(tool_dir: Path) -> None:
    movable = TOOL_URDF.replace(
        '<joint name="jaw_1_joint" type="fixed">',
        '<joint name="jaw_1_joint" type="prismatic">\n    <axis xyz="0 1 0"/>',
    ).replace(
        '<joint name="jaw_2_joint" type="fixed">',
        '<joint name="jaw_2_joint" type="prismatic">\n    <mimic joint="jaw_1_joint" multiplier="-1" offset="0"/>',
    )
    (tool_dir / "gripper_v1.urdf").write_text(movable, encoding="utf-8")

    messages: list[str] = []
    handler_id = logger.add(messages.append, level="WARNING", format="{message}")
    try:
        macro = _macro(_convert_tool(tool_dir))
    finally:
        logger.remove(handler_id)

    assert any("['jaw_1_joint', 'jaw_2_joint']" in m for m in messages)
    assert macro.find("joint[@name='gripper_v1_jaw_1_joint']").get("type") == "prismatic"
    assert macro.find("joint[@name='gripper_v1_jaw_2_joint']/mimic").get("joint") == "gripper_v1_jaw_1_joint"


# --- arm + tool -------------------------------------------------------------------------------


def _assert_single_tree(robot: ET._Element) -> None:
    links = _names(robot, "link")
    assert len(links) == len(set(links)), "duplicate link names"
    joints = _names(robot, "joint")
    assert len(joints) == len(set(joints)), "duplicate joint names"
    materials = {m.get("name"): m for m in robot.iter("material")}
    assert "gripper_v1_Base_1-material" in materials
    assert "Base_1-material" in materials

    children: dict[str, list[str]] = {name: [] for name in links}
    parent_of: dict[str, str] = {}
    for joint in robot.findall("joint"):
        parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
        assert parent in children, f"joint {joint.get('name')} references unknown parent {parent}"
        assert child in children, f"joint {joint.get('name')} references unknown child {child}"
        assert child not in parent_of, f"link {child} has two parents"
        parent_of[child] = parent
        children[parent].append(child)

    roots = [name for name in links if name not in parent_of]
    assert roots == ["Base_1"]
    reachable, stack = set(), list(roots)
    while stack:
        name = stack.pop()
        reachable.add(name)
        stack.extend(children[name])
    assert reachable == set(links)
    assert children["tcp"] == []
    assert parent_of["gripper_v1_Base_1"] == "flange"
    assert parent_of["flange"] == "Stage6_1"


def test_arm_and_tool_merge_manually(arm_xacro: Path, tool_dir: Path) -> None:
    """Expand both macros by hand (no xacro dependency) and check the result is one tree."""
    tool_xacro = _convert_tool(tool_dir, tcp_xyz=(0, 0, 0.12))
    robot = ET.Element("robot", name="robotarm")
    for element in _macro(arm_xacro):
        robot.append(element)
    for element in _macro(tool_xacro):
        for ref in element.iter("parent"):
            if ref.get("link") == "${parent}":
                ref.set("link", "flange")
        robot.append(element)
    assert "${" not in ET.tostring(robot, encoding="unicode")
    _assert_single_tree(robot)


def test_arm_and_tool_merge_with_xacro(tmp_path: Path, arm_xacro: Path, tool_dir: Path) -> None:
    xacro = pytest.importorskip("xacro")
    tool_xacro = _convert_tool(tool_dir, tcp_xyz=(0, 0, 0.12))
    wrapper = _write(
        tmp_path / "robot.urdf.xacro",
        f"""<?xml version="1.0" ?>
<robot name="robotarm" xmlns:xacro="{XACRO_NS}">
  <xacro:include filename="{arm_xacro}"/>
  <xacro:include filename="{tool_xacro}"/>
  <xacro:robotarm_geometry/>
  <xacro:tool parent="flange"/>
</robot>
""",
    )
    expanded = xacro.process_file(str(wrapper)).toxml()
    _assert_single_tree(ET.fromstring(expanded.encode()))  # noqa: S320
