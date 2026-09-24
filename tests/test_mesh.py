"""Tests for local mesh post-processing (visual decimation and convex collision meshes)."""

from __future__ import annotations

import os
from concurrent.futures import ThreadPoolExecutor
from unittest.mock import Mock

import numpy as np
import pytest
import trimesh
from lxml import etree as ET

from onshape_robotics_toolkit import mesh as mesh_module
from onshape_robotics_toolkit.connect import Asset
from onshape_robotics_toolkit.formats import MJCFSerializer, URDFSerializer, process_urdf_meshes
from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.mesh import (
    MeshOptions,
    convex_decomposition,
    convex_hull,
    process_meshes,
    simplify_mesh,
)
from onshape_robotics_toolkit.models.geometry import MeshGeometry
from onshape_robotics_toolkit.models.link import CollisionLink, Link, Material, Origin, VisualLink
from onshape_robotics_toolkit.robot import Robot


def _u_shape() -> trimesh.Trimesh:
    """A concave U-shaped part, the kind of shape a single convex hull over-approximates."""
    base = trimesh.creation.box(extents=(0.3, 0.1, 0.05))
    left = trimesh.creation.box(extents=(0.05, 0.1, 0.2))
    left.apply_translation((-0.125, 0, 0.125))
    right = left.copy()
    right.apply_translation((0.25, 0, 0))
    return trimesh.util.concatenate([base, left, right])


def _write_stl(mesh: trimesh.Trimesh, path) -> str:
    mesh.export(str(path), file_type="stl")
    return str(path)


def test_simplify_mesh_respects_max_faces() -> None:
    sphere = trimesh.creation.icosphere(subdivisions=5)
    assert len(sphere.faces) > 10000

    simplified = simplify_mesh(sphere, 1000)

    assert 0 < len(simplified.faces) <= 1000


def test_simplify_mesh_keeps_small_mesh() -> None:
    box = trimesh.creation.box()
    assert simplify_mesh(box, 1000) is box


def test_simplify_mesh_is_safe_to_call_from_threads() -> None:
    sphere = trimesh.creation.icosphere(subdivisions=6)

    with ThreadPoolExecutor(4) as pool:
        face_counts = list(pool.map(lambda _: len(simplify_mesh(sphere, 5000).faces), range(8)))

    assert all(0 < count <= 5000 for count in face_counts)
    assert len(set(face_counts)) == 1


def test_convex_hull_is_convex_and_bounded() -> None:
    sphere = trimesh.creation.icosphere(subdivisions=4)

    hull = convex_hull(sphere, max_vertices=32)

    assert hull.is_convex
    assert len(hull.vertices) <= 32


def test_convex_decomposition_splits_concave_shape() -> None:
    pytest.importorskip("coacd")

    parts = convex_decomposition(_u_shape(), max_hulls=8, max_vertices=32, concavity=0.05)

    assert 1 < len(parts) <= 8
    assert all(part.is_convex for part in parts)
    assert all(len(part.vertices) <= 32 for part in parts)


def test_process_meshes_writes_visual_and_collision_and_uses_cache(tmp_path, monkeypatch) -> None:
    mesh_dir = tmp_path / "meshes"
    (mesh_dir / "source").mkdir(parents=True)
    source = _write_stl(trimesh.creation.icosphere(subdivisions=5), mesh_dir / "source" / "link.stl")
    visual = str(mesh_dir / "link.stl")
    options = MeshOptions(visual_max_faces=2000, collision_mode="convex_hull", collision_hull_max_vertices=32)

    results = process_meshes([("link", source, visual)], str(mesh_dir), options, max_workers=1)

    assert len(trimesh.load(visual).faces) <= 2000
    assert results["link"] == [str(mesh_dir / "collision" / "link_collision_0.stl")]
    assert trimesh.load(results["link"][0]).is_convex

    # Second run with unchanged source and options must not reprocess anything
    def _fail(*args, **kwargs):
        raise AssertionError("cache miss: mesh was reprocessed")

    monkeypatch.setattr(mesh_module, "ThreadPoolExecutor", _fail)
    assert process_meshes([("link", source, visual)], str(mesh_dir), options) == results


def test_process_meshes_mesh_mode_keeps_visual_as_collision(tmp_path) -> None:
    mesh_dir = tmp_path / "meshes"
    (mesh_dir / "source").mkdir(parents=True)
    source = _write_stl(trimesh.creation.box(), mesh_dir / "source" / "link.stl")
    visual = str(mesh_dir / "link.stl")

    results = process_meshes([("link", source, visual)], str(mesh_dir), MeshOptions(collision_mode="mesh"))

    assert results["link"] == []
    assert os.path.exists(visual)


def test_process_meshes_removes_stale_collision_files(tmp_path) -> None:
    mesh_dir = tmp_path / "meshes"
    (mesh_dir / "source").mkdir(parents=True)
    (mesh_dir / "collision").mkdir()
    stale = mesh_dir / "collision" / "link_collision_7.stl"
    stale.write_text("stale")
    source = _write_stl(trimesh.creation.box(), mesh_dir / "source" / "link.stl")

    process_meshes(
        [("link", source, str(mesh_dir / "link.stl"))],
        str(mesh_dir),
        MeshOptions(collision_mode="convex_hull"),
        max_workers=1,
    )

    assert not stale.exists()


def test_link_accepts_single_collision_for_backwards_compatibility() -> None:
    collision = CollisionLink(name="c", origin=Origin.zero_origin(), geometry=MeshGeometry("a.stl"))

    assert Link(name="l", collision=collision).collision == [collision]
    assert Link(name="l", collision=None).collision == []


def test_link_with_multiple_collisions_round_trips_through_urdf() -> None:
    link = Link(
        name="l",
        collision=[
            CollisionLink(name=f"l_collision_{i}", origin=Origin.zero_origin(), geometry=MeshGeometry(f"c{i}.stl"))
            for i in range(3)
        ],
    )

    xml = link.to_xml()
    assert len(xml.findall("collision")) == 3

    parsed = Link.from_xml(xml)
    assert [c.geometry.filename for c in parsed.collision] == ["c0.stl", "c1.stl", "c2.stl"]


@pytest.fixture
def one_link_robot(tmp_path):
    """A single-link robot whose mesh was downloaded by an earlier export, with no Onshape client."""
    mesh_dir = tmp_path / "meshes"
    mesh_dir.mkdir()
    _write_stl(_u_shape().subdivide().subdivide(), mesh_dir / "base.stl")

    asset = Asset(file_name="base.stl", client=None, mesh_dir=str(mesh_dir))
    link = Link(
        name="base",
        visual=VisualLink(
            name="base_visual",
            origin=Origin.zero_origin(),
            geometry=MeshGeometry("x"),
            material=Material(name="base_material", color=(0.5, 0.5, 0.5, 1.0)),
        ),
        collision=CollisionLink(name="base_collision", origin=Origin.zero_origin(), geometry=MeshGeometry("x")),
    )
    graph = Mock(spec=KinematicGraph)
    robot = Robot(kinematic_graph=graph, name="test_robot")
    robot.add_node("base", data=link, asset=asset, world_to_link_tf=np.eye(4))
    return robot, mesh_dir


def test_save_with_mesh_options_uses_local_meshes_only(tmp_path, one_link_robot, monkeypatch) -> None:
    robot, mesh_dir = one_link_robot

    async def _no_download(*args, **kwargs):
        raise AssertionError("mesh processing must not download anything")

    monkeypatch.setattr(Asset, "download", _no_download)

    options = MeshOptions(visual_max_faces=100, collision_mode="convex_hull")
    URDFSerializer().save(robot, str(tmp_path / "robot.urdf"), download_assets=False, mesh_options=options)

    assert (mesh_dir / "source" / "base.stl").exists()
    assert len(trimesh.load(str(mesh_dir / "base.stl")).faces) <= 100

    urdf = ET.parse(str(tmp_path / "robot.urdf")).getroot()  # noqa: S320
    link = urdf.find("link")
    assert link.find("visual/geometry/mesh").get("filename") == "meshes/base.stl"
    collision_files = [m.get("filename") for m in link.findall("collision/geometry/mesh")]
    assert collision_files == ["meshes/collision/base_collision_0.stl"]


def test_mjcf_save_with_mesh_options_declares_collision_mesh_assets(tmp_path, one_link_robot) -> None:
    pytest.importorskip("coacd")
    robot, _ = one_link_robot

    MJCFSerializer().save(
        robot,
        str(tmp_path / "robot.xml"),
        download_assets=False,
        mesh_options=MeshOptions(collision_mode="convex_decomposition", collision_max_hulls=4),
    )

    mjcf = ET.parse(str(tmp_path / "robot.xml")).getroot()  # noqa: S320
    asset_names = [m.get("name") for m in mjcf.findall("asset/mesh")]
    assert len(asset_names) == len(set(asset_names))
    collision_names = [n for n in asset_names if n.startswith("base_collision_")]
    assert len(collision_names) > 1
    geom_meshes = {g.get("mesh") for g in mjcf.iter("geom")}
    assert set(collision_names) <= geom_meshes


def test_process_urdf_meshes_rewrites_existing_xacro(tmp_path) -> None:
    mesh_dir = tmp_path / "meshes"
    mesh_dir.mkdir()
    _write_stl(trimesh.creation.icosphere(subdivisions=4), mesh_dir / "Base_1.stl")
    xacro = tmp_path / "robot.xacro"
    xacro.write_text(
        '<?xml version="1.0" ?>\n'
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:macro name="geometry">'
        '<link name="Base_1">'
        '<visual><origin xyz="0 0 0" rpy="0 0 0"/>'
        '<geometry><mesh filename="package://pkg/meshes/Base_1.stl"/></geometry></visual>'
        '<collision><origin xyz="0 0 0" rpy="0 0 0"/>'
        '<geometry><mesh filename="package://pkg/meshes/Base_1.stl"/></geometry></collision>'
        '</link><link name="tcp"/></xacro:macro></robot>'
    )
    options = MeshOptions(visual_max_faces=500, collision_mode="convex_hull")

    for _ in range(2):  # re-running must not duplicate collision elements
        process_urdf_meshes(str(xacro), str(mesh_dir), options)

    root = ET.parse(str(xacro)).getroot()  # noqa: S320
    link = next(link for link in root.iter("link") if link.get("name") == "Base_1")
    assert link.find("visual/geometry/mesh").get("filename") == "package://pkg/meshes/Base_1.stl"
    assert [m.get("filename") for m in link.findall("collision/geometry/mesh")] == [
        "package://pkg/meshes/collision/Base_1_collision_0.stl"
    ]
    assert (mesh_dir / "source" / "Base_1.stl").exists()
    assert len(trimesh.load(str(mesh_dir / "Base_1.stl")).faces) <= 500
    assert trimesh.load(str(mesh_dir / "collision" / "Base_1_collision_0.stl")).is_convex

    # Switching back to collision_mode="mesh" points collision at the visual mesh again
    process_urdf_meshes(str(xacro), str(mesh_dir), MeshOptions(collision_mode="mesh"))
    link = next(link for link in ET.parse(str(xacro)).getroot().iter("link") if link.get("name") == "Base_1")  # noqa: S320
    assert [m.get("filename") for m in link.findall("collision/geometry/mesh")] == ["package://pkg/meshes/Base_1.stl"]
