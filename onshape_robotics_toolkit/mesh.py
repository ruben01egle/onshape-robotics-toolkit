"""
This module contains functions for transforming meshes and inertia matrices, and for
post-processing downloaded meshes into lighter visual meshes and convex collision meshes.

All post-processing runs locally on already-downloaded STL files; it never talks to Onshape.

"""

import hashlib
import json
import os
import shutil
import threading
from concurrent.futures import ThreadPoolExecutor
from functools import partial
from typing import Any, Literal, Optional

import numpy as np
from loguru import logger
from numpy.typing import NDArray
from pydantic import BaseModel, Field
from stl.mesh import Mesh

__all__ = [
    "MeshOptions",
    "convex_decomposition",
    "convex_hull",
    "process_link_mesh",
    "process_meshes",
    "simplify_mesh",
    "transform_inertia_matrix",
    "transform_mesh",
    "transform_vectors",
]

SOURCE_MESH_DIR = "source"
COLLISION_MESH_DIR = "collision"
MESH_CACHE_FILE = ".mesh_cache.json"

_SIMPLIFY_LOCK = threading.Lock()


class MeshOptions(BaseModel):
    """
    Local post-processing applied to downloaded meshes.

    When passed to ``RobotSerializer.save(..., mesh_options=...)``, the full-resolution
    download is kept in ``<mesh_dir>/source/``, the visual mesh in ``<mesh_dir>/`` is
    derived from it, and separate collision meshes are written to ``<mesh_dir>/collision/``.

    Attributes:
        visual_max_faces: Decimate each visual mesh to at most this many triangles.
            ``None`` keeps the visual at full resolution.
        collision_mode: How collision geometry is generated:
            ``"mesh"`` reuses the visual mesh (legacy behavior),
            ``"convex_hull"`` uses a single convex hull per link,
            ``"convex_decomposition"`` splits each link into several convex hulls using CoACD
            (requires the ``coacd`` package, falls back to ``"convex_hull"`` if missing).
        collision_max_hulls: Maximum number of convex hulls per link (convex_decomposition).
        collision_hull_max_vertices: Maximum number of vertices per convex hull.
        collision_concavity: CoACD concavity threshold; lower is tighter but yields more hulls.
    """

    visual_max_faces: Optional[int] = Field(default=None, gt=0)
    collision_mode: Literal["mesh", "convex_hull", "convex_decomposition"] = "convex_decomposition"
    collision_max_hulls: int = Field(default=16, gt=0)
    collision_hull_max_vertices: int = Field(default=64, ge=4)
    collision_concavity: float = Field(default=0.05, gt=0)


def simplify_mesh(mesh: Any, max_faces: int) -> Any:
    """
    Decimate a trimesh mesh to at most ``max_faces`` triangles using quadric edge collapse.

    Args:
        mesh: trimesh.Trimesh to simplify
        max_faces: Target maximum number of faces

    Returns:
        The simplified trimesh.Trimesh (or the input if it is already small enough)
    """
    import fast_simplification  # type: ignore[import-untyped]
    import trimesh

    if len(mesh.faces) <= max_faces:
        return mesh

    vertices = np.asarray(mesh.vertices, dtype=np.float64)
    faces = np.asarray(mesh.faces, dtype=np.int64)
    # CAD meshes are many disconnected, open bodies; a single pass can stall well above the
    # target because boundary edges are not collapsed, so repeat while it still makes progress.
    for _ in range(3):
        if len(faces) <= max_faces:
            break
        # fast_simplification is not thread-safe (concurrent calls can return empty meshes)
        with _SIMPLIFY_LOCK:
            new_vertices, new_faces = fast_simplification.simplify(
                vertices, faces, target_reduction=1.0 - max_faces / len(faces)
            )[:2]
        if len(new_faces) == 0 or len(new_faces) > 0.99 * len(faces):
            break
        vertices, faces = new_vertices, new_faces

    if len(faces) > max_faces:
        logger.debug(f"Decimation stopped at {len(faces)} faces (target {max_faces})")
    return trimesh.Trimesh(vertices=vertices, faces=faces)


def convex_hull(mesh: Any, max_vertices: int) -> Any:
    """
    Compute the convex hull of a mesh, reduced to at most ``max_vertices`` vertices.

    Args:
        mesh: trimesh.Trimesh to wrap
        max_vertices: Maximum number of hull vertices

    Returns:
        A convex trimesh.Trimesh
    """
    import trimesh

    hull = mesh.convex_hull
    if len(hull.vertices) <= max_vertices:
        return hull

    # Keep the hull vertices that best span the shape (farthest-point sampling), then re-hull
    points = np.asarray(hull.vertices)
    selected = [int(np.argmax(np.linalg.norm(points - points.mean(axis=0), axis=1)))]
    distances = np.linalg.norm(points - points[selected[0]], axis=1)
    for _ in range(max_vertices - 1):
        selected.append(int(np.argmax(distances)))
        distances = np.minimum(distances, np.linalg.norm(points - points[selected[-1]], axis=1))
    return trimesh.Trimesh(vertices=points[selected]).convex_hull


def convex_decomposition(mesh: Any, max_hulls: int, max_vertices: int, concavity: float) -> list[Any]:
    """
    Split a mesh into convex hulls using CoACD.

    Args:
        mesh: trimesh.Trimesh to decompose
        max_hulls: Maximum number of convex hulls
        max_vertices: Maximum number of vertices per hull
        concavity: CoACD concavity threshold

    Returns:
        List of convex trimesh.Trimesh parts
    """
    import coacd  # type: ignore[import-untyped]
    import trimesh

    coacd.set_log_level("error")
    parts = coacd.run_coacd(
        coacd.Mesh(np.asarray(mesh.vertices), np.asarray(mesh.faces)),
        threshold=concavity,
        max_convex_hull=max_hulls,
        decimate=True,
        max_ch_vertex=max_vertices,
    )
    return [trimesh.Trimesh(vertices=vertices, faces=faces).convex_hull for vertices, faces in parts]


def process_link_mesh(
    source_path: str,
    visual_path: str,
    collision_dir: str,
    name: str,
    options: MeshOptions,
) -> list[str]:
    """
    Derive the visual mesh and the collision meshes of one link from its full-resolution source mesh.

    Args:
        source_path: Full-resolution STL (already transformed into the link frame)
        visual_path: Where to write the visual STL
        collision_dir: Directory to write the collision STLs to
        name: Base name used for the collision files (``<name>_collision_<i>.stl``)
        options: Processing options

    Returns:
        Absolute paths of the written collision meshes; empty if collision should reuse the visual mesh.
    """
    import trimesh

    mesh: Any = trimesh.load(source_path, file_type="stl", force="mesh")

    if options.visual_max_faces is not None and len(mesh.faces) > options.visual_max_faces:
        simplify_mesh(mesh, options.visual_max_faces).export(visual_path, file_type="stl")
    elif os.path.abspath(source_path) != os.path.abspath(visual_path):
        shutil.copyfile(source_path, visual_path)

    for stale in _existing_collision_files(collision_dir, name):
        os.remove(stale)

    if options.collision_mode == "mesh" or len(mesh.faces) == 0:
        return []

    try:
        if options.collision_mode == "convex_decomposition":
            parts = convex_decomposition(
                mesh,
                max_hulls=options.collision_max_hulls,
                max_vertices=options.collision_hull_max_vertices,
                concavity=options.collision_concavity,
            )
        else:
            parts = [convex_hull(mesh, options.collision_hull_max_vertices)]
    except Exception as e:
        logger.warning(f"Collision mesh generation failed for {name}, falling back to the visual mesh: {e}")
        return []

    os.makedirs(collision_dir, exist_ok=True)
    paths = []
    for i, part in enumerate(parts):
        path = os.path.join(collision_dir, f"{name}_collision_{i}.stl")
        part.export(path, file_type="stl")
        paths.append(path)
    return paths


def _existing_collision_files(collision_dir: str, name: str) -> list[str]:
    if not os.path.isdir(collision_dir):
        return []
    prefix = f"{name}_collision_"
    return sorted(
        os.path.join(collision_dir, f)
        for f in os.listdir(collision_dir)
        if f.startswith(prefix) and f.endswith(".stl") and f[len(prefix) : -len(".stl")].isdigit()
    )


def _cache_key(source_path: str, options: MeshOptions) -> str:
    digest = hashlib.sha256()
    with open(source_path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            digest.update(chunk)
    digest.update(options.model_dump_json().encode())
    return digest.hexdigest()


def process_meshes(
    jobs: list[tuple[str, str, str]],
    mesh_dir: str,
    options: MeshOptions,
    max_workers: Optional[int] = None,
) -> dict[str, list[str]]:
    """
    Process the meshes of several links in parallel, skipping links whose source mesh
    and options are unchanged since the last run.

    Threads are used rather than processes: the heavy native work (CoACD, decimation, qhull)
    releases the GIL, and worker processes started via spawn/forkserver would re-import the
    user's export script, which could re-run the export and its API calls.

    Args:
        jobs: ``(name, source_path, visual_path)`` per link
        mesh_dir: Mesh directory; collision meshes go to ``<mesh_dir>/collision`` and the
            cache to ``<mesh_dir>/.mesh_cache.json``
        options: Processing options
        max_workers: Worker threads (defaults to the CPU count)

    Returns:
        Mapping from link name to the absolute paths of its collision meshes.
    """
    if options.collision_mode == "convex_decomposition":
        try:
            import coacd  # noqa: F401
        except ImportError:
            logger.warning(
                "collision_mode='convex_decomposition' requires the 'coacd' package "
                "(pip install onshape-robotics-toolkit[collision]); falling back to 'convex_hull'."
            )
            options = options.model_copy(update={"collision_mode": "convex_hull"})

    collision_dir = os.path.join(mesh_dir, COLLISION_MESH_DIR)
    cache_path = os.path.join(mesh_dir, MESH_CACHE_FILE)
    try:
        with open(cache_path, encoding="utf-8") as f:
            cache: dict[str, Any] = json.load(f)
    except (OSError, ValueError):
        cache = {}

    results: dict[str, list[str]] = {}
    pending: list[tuple[str, str, str, str]] = []
    for name, source_path, visual_path in jobs:
        key = _cache_key(source_path, options)
        entry = cache.get(name)
        if (
            entry
            and entry.get("key") == key
            and os.path.exists(visual_path)
            and entry.get("visual_size") == os.path.getsize(visual_path)
        ):
            cached_paths = [os.path.join(collision_dir, f) for f in entry.get("collision", [])]
            if all(os.path.exists(p) for p in cached_paths):
                results[name] = cached_paths
                logger.debug(f"Mesh cache hit for {name}")
                continue
        pending.append((name, source_path, visual_path, key))

    if pending:
        logger.info(f"Processing {len(pending)} mesh(es) locally ({options.collision_mode})")
        with ThreadPoolExecutor(max_workers=max_workers or os.cpu_count()) as pool:
            futures = {
                name: (key, pool.submit(process_link_mesh, source_path, visual_path, collision_dir, name, options))
                for name, source_path, visual_path, key in pending
            }
            visual_paths = {name: visual_path for name, _, visual_path, _ in pending}
            for name, (key, future) in futures.items():
                try:
                    paths = future.result()
                except Exception as e:
                    logger.error(f"Failed to process mesh {name}: {e}")
                    cache.pop(name, None)
                    continue
                results[name] = paths
                cache[name] = {
                    "key": key,
                    "visual_size": os.path.getsize(visual_paths[name]),
                    "collision": [os.path.basename(p) for p in paths],
                }

        with open(cache_path, "w", encoding="utf-8") as f:
            json.dump(cache, f, indent=2)

    return results


def transform_vectors(
    vectors: NDArray[np.floating], rotation: NDArray[np.floating], translation: NDArray[np.floating]
) -> NDArray[np.floating]:
    """
    Apply a transformation matrix to a set of vectors.

    Args:
        vectors: Array of vectors to use for transformation
        rotation: Rotation matrix to apply to the vectors
        translation: Translation matrix to apply to the vectors

    Returns:
        Array of transformed vectors
    """

    result: NDArray[np.floating] = np.dot(vectors, rotation.T) + translation * len(vectors)
    return result


def transform_mesh(mesh: Mesh, transform: np.ndarray) -> Mesh:
    """
    Apply a transformation matrix to an STL mesh.

    Args:
        mesh: STL mesh to use for transformation
        transform: Transformation matrix to apply to the mesh

    Returns:
        Transformed STL mesh

    Examples:
        >>> mesh = Mesh.from_file("mesh.stl")
        >>> transform = np.eye(4)
        >>> transform_mesh(mesh, transform)
    """

    _transform_vectors = partial(
        transform_vectors, rotation=transform[:3, :3], translation=transform[0:3, 3:4].T.tolist()
    )

    mesh.v0 = _transform_vectors(mesh.v0)
    mesh.v1 = _transform_vectors(mesh.v1)
    mesh.v2 = _transform_vectors(mesh.v2)
    mesh.normals = _transform_vectors(mesh.normals)

    return mesh


def transform_inertia_matrix(
    inertia_matrix: NDArray[np.floating], rotation: NDArray[np.floating]
) -> NDArray[np.floating]:
    """
    Transform an inertia matrix

    Args:
        inertia_matrix: Inertia matrix to use for transformation
        rotation: Rotation matrix to apply to the inertia matrix

    Returns:
        Transformed inertia matrix
    """

    result: NDArray[np.floating] = rotation @ inertia_matrix @ rotation.T
    return result
