"""
Standalone mesh post-processing for an existing URDF or xacro file.

Runs the same local processing as ``RobotSerializer.save(..., mesh_options=...)``
(visual decimation, convex collision meshes) on a robot description that was exported
earlier, without an Onshape client and without making any API calls.
"""

from __future__ import annotations

import os
import shutil
from pathlib import Path

from loguru import logger
from lxml import etree as ET

from onshape_robotics_toolkit.mesh import COLLISION_MESH_DIR, SOURCE_MESH_DIR, MeshOptions, process_meshes


def process_urdf_meshes(
    urdf_path: str,
    mesh_dir: str,
    mesh_options: MeshOptions,
    output_path: str | None = None,
) -> dict[str, list[str]]:
    """
    Post-process the meshes of an existing URDF/xacro file and rewrite its collision elements.

    For every ``<link>`` whose ``<visual>`` references a mesh found in ``mesh_dir``:

    - the full-resolution mesh is kept in ``<mesh_dir>/source/`` (copied from the visual mesh
      on the first run, so re-running with different options always starts from full resolution),
    - the visual mesh is replaced by its decimated version (if ``visual_max_faces`` is set),
    - the link's ``<collision>`` elements are replaced by one element per generated collision
      mesh in ``<mesh_dir>/collision/``, using the same path prefix as the visual mesh
      (so ``package://`` paths keep working).

    Links are matched by mesh file name, so ``package://<pkg>/meshes/...`` and relative paths
    both work. Running it again is safe; unchanged links are served from the mesh cache.

    **Example:**
        >>> from onshape_robotics_toolkit.formats import process_urdf_meshes
        >>> from onshape_robotics_toolkit.mesh import MeshOptions
        >>> process_urdf_meshes(
        ...     "urdf/robotarm_geometry.xacro",
        ...     mesh_dir="meshes",
        ...     mesh_options=MeshOptions(visual_max_faces=100_000, collision_mode="convex_hull"),
        ... )

    Args:
        urdf_path: URDF or xacro file to read.
        mesh_dir: Directory holding the link meshes referenced by the file.
        mesh_options: Local mesh processing options.
        output_path: Where to write the updated file. Defaults to overwriting ``urdf_path``.

    Returns:
        Mapping from mesh name to the absolute paths of its collision meshes.
    """
    parser = ET.XMLParser(remove_blank_text=True)
    tree = ET.parse(urdf_path, parser)  # noqa: S320
    root = tree.getroot()

    # (link element, visual mesh filename as written in the file, mesh name)
    targets: list[tuple[ET._Element, str, str]] = []
    jobs: dict[str, tuple[str, str, str]] = {}
    for link in root.iter("link"):
        mesh = link.find("visual/geometry/mesh")
        filename = mesh.get("filename") if mesh is not None else None
        if not filename:
            continue

        basename = filename.replace("\\", "/").rsplit("/", 1)[-1]
        name = os.path.splitext(basename)[0]
        visual_path = os.path.join(mesh_dir, basename)
        source_path = os.path.join(mesh_dir, SOURCE_MESH_DIR, basename)

        if not os.path.exists(source_path):
            if not os.path.exists(visual_path):
                logger.warning(f"Mesh {basename} of link {link.get('name')} not found in {mesh_dir}, skipping")
                continue
            os.makedirs(os.path.dirname(source_path), exist_ok=True)
            shutil.copyfile(visual_path, source_path)

        targets.append((link, filename, name))
        jobs[name] = (name, source_path, visual_path)

    results = process_meshes(list(jobs.values()), mesh_dir, mesh_options)

    for link, visual_filename, name in targets:
        prefix = visual_filename.replace("\\", "/").rsplit("/", 1)[0] + "/" if "/" in visual_filename else ""
        collision_filenames = [
            f"{prefix}{COLLISION_MESH_DIR}/{os.path.basename(path)}" for path in results.get(name, [])
        ]
        if not collision_filenames:
            # Collision reuses the visual mesh; only rewrite links still pointing at generated meshes
            generated = f"/{COLLISION_MESH_DIR}/"
            if not any(generated in (m.get("filename") or "") for m in link.findall("collision/geometry/mesh")):
                continue
            collision_filenames = [visual_filename]

        for old in link.findall("collision"):
            link.remove(old)

        insert_at = next(i for i, child in enumerate(link) if child.tag == "visual") + 1
        for i, filename in enumerate(collision_filenames):
            collision = ET.Element("collision", name=f"{link.get('name')}_collision_{i}")
            ET.SubElement(collision, "origin", xyz="0 0 0", rpy="0 0 0")
            geometry = ET.SubElement(collision, "geometry")
            ET.SubElement(geometry, "mesh", filename=filename)
            link.insert(insert_at + i, collision)

    ET.indent(root, space="    ")
    content = '<?xml version="1.0" ?>\n' + ET.tostring(root, pretty_print=True, encoding="unicode")
    Path(output_path or urdf_path).write_text(content, encoding="utf-8")
    logger.info(f"Processed meshes of {len(targets)} link(s) in {output_path or urdf_path}")
    return results
