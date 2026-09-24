# onshape-robotics-toolkit

[![License](https://img.shields.io/github/license/neurobionics/onshape-robotics-toolkit)](https://github.com/neurobionics/onshape-robotics-toolkit/blob/main/LICENSE)
[![codecov](https://codecov.io/gh/neurobionics/onshape-robotics-toolkit/branch/main/graph/badge.svg)](https://codecov.io/gh/neurobionics/onshape-robotics-toolkit)
[![DOI](https://zenodo.org/badge/865051944.svg)](https://doi.org/10.5281/zenodo.14617407)
[![Docs](https://img.shields.io/badge/docs-mkdocs-blue)](https://neurobionics.github.io/onshape-robotics-toolkit/)
[![PyPI Downloads](https://static.pepy.tech/personalized-badge/onshape-robotics-toolkit?period=total&units=INTERNATIONAL_SYSTEM&left_color=BLACK&right_color=GREEN&left_text=downloads)](https://pepy.tech/projects/onshape-robotics-toolkit)

<img src="https://github.com/neurobionics/onshape-robotics-toolkit/blob/2fd1cc6a6d67419169d2be750a421bccd0940247/docs/tutorials/export/export-header.gif" alt="Header" style="width: 100%;">

The `onshape-robotics-toolkit` is a feature-rich Python library that significantly extends the capabilities of Onshape's web-based CAD platform. The library provides a comprehensive API for automating robot design tasks, including solid model manipulation, robot assembly management, graph-based visualizations, and exporting CAD assemblies to URDF files for simulation and control. Intended as a resource for the robotics community, this library leverages Onshape's REST API to facilitate advanced workflows that bridge CAD design and robotics applications.

&nbsp;
This library was inspired by <a href="https://github.com/Rhoban/onshape-to-robot" target="_blank">onshape-to-robot</a>, a tool renowned for its streamlined approach to URDF generation. While onshape-to-robot library focuses on predefined workflows and design-time considerations, the `onshape-robotics-toolkit` library offers greater flexibility. It provides access to nearly all of Onshape's REST API calls, enabling headless manipulation, detailed analysis, and seamless export of CAD assemblies. Users can programmatically edit variable studios, generate graph-based visualizations, and export URDF files tailored to their specific needs—all without being restricted by rigid workflows or naming conventions. By removing these constraints, the `onshape-robotics-toolkit` library empowers the robotics and CAD communities to create custom solutions for algorithmic design, optimization, and automation.

&nbsp;

# Key Features of `onshape-robotics-toolkit`

The `onshape-robotics-toolkit` library is designed for users seeking a scalable, versatile API that empowers innovative robot design and control workflows. By integrating Onshape into algorithmic processes such as design optimization and automation, it unlocks the full potential of Onshape's cloud-based CAD system, fostering creativity and efficiency in robotics and beyond.

&nbsp;

| Feature                              | `onshape-robotics-toolkit`           | `onshape-to-robot`                      |
| ------------------------------------ | ------------------------------------ | --------------------------------------- |
| **Workflow Flexibility**             | ✅ Open-ended and customizable       | ❌ Predefined and rigid                 |
| **Design-Time Considerations**       | ✅ None                              | ❌ Requires specific naming conventions |
| **Custom URDF Workflow**             | ✅ Supports any assembly             | ❌ Limited by design rules              |
| **Variable Studio Editing**          | ✅ Yes                               | ❌ No                                   |
| **Ease of Setup**                    | ❌ Moderate (requires python coding) | ✅ Easy (no coding required)            |
| **Headless Integration**             | ✅ Yes (e.g., optimization)          | ❌ No out-of-the-box support            |
| **Access to Full Onshape API**       | ✅ Yes                               | ❌ Limited                              |
| **Graph Visualization and Analysis** | ✅ Supports graph generation         | ❌ Not supported                        |

## Prerequisites

Before you begin, ensure you have the following:

- <a href="https://www.python.org/downloads/release/python-3100/" target="_blank">Python 3.10</a> or higher installed on your machine.
- <a href="https://www.onshape.com/en/" target="_blank">An Onshape account</a> if you don't already have one.
- <a href="https://onshape-public.github.io/docs/auth/apikeys/" target="_blank">Onshape API keys (access key and secret key)</a>

> **Note:** Before using this tool (including making API calls or using it for research), ensure you have read and comply with [Onshape's Terms of Use](https://onshape-public.github.io/docs/auth/limits/#terms-of-use) and be aware of [Onshape's API limits and usage policies](https://onshape-public.github.io/docs/auth/limits/) to avoid exceeding rate limits.

## Installation

You can install `onshape-robotics-toolkit` using `pip`, which is the easiest way to install it and is the recommended method for most users.

```sh
pip install onshape-robotics-toolkit
```

If you want to install from source, you'll need to install [`uv`](https://docs.astral.sh/uv/getting-started/installation/) and [`git`](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git) first. Then, you can clone the repository and install the package.

```sh
git clone https://github.com/neurobionics/onshape-robotics-toolkit.git
cd onshape-robotics-toolkit
uv sync
```

## URDF Export Notes

A few behaviors of the CAD → URDF export pipeline are worth calling out explicitly:

- **Link color.** `Robot.from_graph()` accepts a `uniform_link_color` parameter (default
  `(0.5, 0.5, 0.5, 1.0)`), applied to every link's material so exported robots don't get an
  arbitrary rainbow of per-part colors. Pass `uniform_link_color=None` to opt back into the
  legacy behavior of picking a random color per part from the built-in `Colors` palette.
- **Joint axis convention.** Every revolute, prismatic, and ball-joint sub-joint is emitted
  with `<axis xyz="0 0 1"/>` (previously `0 0 -1`). Joint limits are computed consistently
  relative to that `+Z` axis, so `<limit lower=".../" upper="..."/>` values already reflect
  the real Onshape mate limits - no manual sign-flipping is needed downstream.
- **Origin noise snapping.** `xyz`/`rpy` components of `<origin>` tags (and `<axis>` tags)
  that are smaller than `1e-6` in magnitude are snapped to exact `0`, cleaning up floating-point
  noise from matrix decomposition (e.g. `-4.3121451e-14`). This snapping deliberately does
  **not** apply to inertia tensor components (`ixx`, `ixy`, ..., `izz`) or mass values, which
  keep full precision even when genuinely tiny.
- **Root selection via a named mate-to-origin.** `KinematicGraph.from_cad()` accepts a
  `root_mate_name` parameter, which supersedes the older `use_user_defined_root` /
  `.fixed`-occurrence workflow for choosing the kinematic root. Instead of adding a
  throwaway dummy part, marking it fixed, and mating it to your real base (`Base`) purely to
  give the toolkit something to anchor on, you can now create a mate connector directly from
  the top-level assembly's own **Origin** (Onshape supports this natively - no dummy part
  needed) and add a **FASTENED** mate between that connector and a connector placed inside
  `Base`, naming the mate something recognizable (e.g. `root_mate`):

  ```python
  graph = KinematicGraph.from_cad(cad, root_mate_name="root_mate")
  ```

  This resolves the root directly from that mate's data (the same `MateFeatureData` shape
  used for every other joint), using the mate's own coordinate system to place the root
  link's frame. If `root_mate_name` is given and can't be resolved (no such mate, not a
  `FASTENED` mate, or neither side is an assembly-origin connector), this raises immediately
  rather than silently falling back to auto-detection - avoiding wasted API calls computing
  the wrong robot. `root_mate_name=None` (the default) preserves the original
  `use_user_defined_root` / centrality-based behavior unchanged.

## Lighter Visual Meshes and Convex Collision Meshes

By default every link's `<visual>` and `<collision>` point at the same full-resolution STL
from Onshape, which is heavy to render (RViz) and slow for collision checking in motion
planners. Passing `mesh_options` to `save()` post-processes the meshes **locally**:

```python
from onshape_robotics_toolkit.mesh import MeshOptions

URDFSerializer().save(
    robot,
    "output/robot.urdf",
    download_assets=True,          # or False to reuse meshes from an earlier export
    mesh_options=MeshOptions(
        visual_max_faces=100_000,                # decimate visuals; None = keep full resolution
        collision_mode="convex_decomposition",   # "convex_hull" | "convex_decomposition" | "mesh"
        collision_max_hulls=16,                  # convex pieces per link (decomposition only)
        collision_hull_max_vertices=64,          # vertices per convex piece
        collision_concavity=0.05,                # lower = tighter fit, more pieces
    ),
)
```

- **No extra Onshape API calls.** The download requests are unchanged: one STL request per
  part / one export per rigid subassembly, and each subassembly is still a single fused mesh.
  All processing happens on the downloaded files. With `download_assets=False` it runs
  entirely offline on the meshes already on disk (existing `meshes/<link>.stl` files from an
  earlier export are picked up as the source automatically).
- **Output layout** (inside the mesh directory):
  - `source/<link>.stl` - full-resolution download, kept so options can be changed later
    without re-downloading
  - `<link>.stl` - visual mesh (decimated if `visual_max_faces` is set)
  - `collision/<link>_collision_<i>.stl` - convex collision pieces; each becomes its own
    `<collision>` element (URDF) / `geom` (MJCF)
  - `.mesh_cache.json` - links whose source mesh and options are unchanged are skipped on
    the next export
- **Choosing `visual_max_faces`.** Onshape meshes of multi-part subassemblies are many
  small open bodies (screws, washers, ...), and quadric decimation degrades them quickly
  below ~100k faces. Measured on a ~250-460k-face robot-arm link, the 99th-percentile surface
  deviation was ~1-3 mm at 200k faces, ~3-9 mm at 100k and ~10-18 mm at 60k. 100k faces
  per link is a good default for RViz; decimation is best-effort and may stop slightly above
  the target on such meshes.
- **Collision modes:**
  - `"convex_hull"`: one convex hull per link. Instant, but fills concave gaps (e.g. the
    inside of a bracket or the space between gripper fingers).
  - `"convex_decomposition"`: splits each link into several convex hulls with
    [CoACD](https://github.com/SarahWeiii/CoACD). Tight fit for complex multi-part links;
    takes seconds to minutes per link at export time (links are processed in parallel and
    cached; ~4 min for a 7-link arm on first export, instant afterwards). Needs the optional extra: `pip install "onshape-robotics-toolkit[collision]"`
    (for a git dependency: `onshape-robotics-toolkit[collision] @ git+...`). Without it,
    this mode falls back to `"convex_hull"` with a warning.
  - `"mesh"`: legacy behavior, collision reuses the visual mesh.
- **Planner notes.** Low-poly convex pieces are the fast path in FCL (MoveIt) and
  hpp-fcl/coal (Pinocchio). In Pinocchio, call `buildConvexRepresentation(False)` on each
  collision geometry and use its `.convex` to get GJK instead of mesh-mesh BVH checks. In
  MoveIt, also generate an SRDF that disables collision checks between adjacent /
  never-colliding links - that is often as large a speedup as the mesh simplification.
- Without `mesh_options` (the default), export behaves exactly as before.

**Standalone post-processing of an existing export.** `process_urdf_meshes` runs the same
processing directly on a URDF or xacro file and its mesh folder - no `Client`, no `Robot`,
no API calls. It replaces each link's `<collision>` elements (keeping the visual mesh's
path prefix, so `package://` paths keep working) and can be re-run with other options,
since it always starts from the full-resolution copy in `source/`
(see [`examples/postprocess/main.py`](examples/postprocess/main.py)):

```python
from onshape_robotics_toolkit.formats import process_urdf_meshes
from onshape_robotics_toolkit.mesh import MeshOptions

process_urdf_meshes(
    "output/robotarm.urdf",          # or e.g. "urdf/robotarm_geometry.xacro"
    mesh_dir="output/meshes",
    mesh_options=MeshOptions(visual_max_faces=100_000, collision_mode="convex_decomposition"),
)
```

When shipping the meshes in a ROS package, `source/` (full-resolution copies) is not
referenced by the URDF and can be left out of the install / version control.

## Using This Fork From Another Repo

This fork (`ruben01egle/onshape-robotics-toolkit`) carries fixes and features not yet
upstream (see `CHANGES.md`). The recommended way to consume it from a separate repo -
e.g. one that holds a finished robot description and just needs to regenerate its URDF
occasionally - is a plain **git dependency** pinned to this fork's `main` branch. No
submodule, no private package index required.

1. **Push this fork's changes to `origin/main`** (`git@github.com:ruben01egle/onshape-robotics-toolkit.git`)
   first - a git dependency can only install what's actually been pushed.

2. **In the destination repo**, add the dependency pointing at this fork instead of PyPI.

   With `uv` (recommended, since this project uses `uv`):

   ```toml
   [project]
   dependencies = [
       "onshape-robotics-toolkit",
   ]

   [tool.uv.sources]
   onshape-robotics-toolkit = { git = "https://github.com/ruben01egle/onshape-robotics-toolkit.git", branch = "main" }
   ```

   then `uv sync`.

   With plain `pip` / `requirements.txt`:

   ```
   onshape-robotics-toolkit @ git+https://github.com/ruben01egle/onshape-robotics-toolkit.git@main
   ```

3. **Pulling in new fork commits later** - a branch pin still resolves to one locked
   commit until you explicitly refresh it:

   - `uv`: `uv lock --upgrade-package onshape-robotics-toolkit && uv sync`
   - `pip`: `pip install -U --force-reinstall git+https://github.com/ruben01egle/onshape-robotics-toolkit.git@main`

4. **Move the URDF-generation script itself into the destination repo.** `robotarm_urdf.py`
   in this repo is a self-contained example - copy it (and the `.env` file it reads via
   `Client(env=".env")`) into the destination repo's root. It only needs the package
   installed as above; `xacro_export.py` does **not** need to move separately, since its
   logic now lives in the package as `onshape_robotics_toolkit.formats.xacro` (imported
   via `from onshape_robotics_toolkit.formats.xacro import convert_urdf_to_xacro`).

5. **Run it from the destination repo's root.** The script writes to CWD-relative paths
   (`output/robotarm.urdf`, `output/meshes/`, `robotarm.log`), so it should be invoked
   from wherever you want that `output/` directory to land.

## Documentation

The documentation is available at [https://neurobionics.github.io/onshape-robotics-toolkit/](https://neurobionics.github.io/onshape-robotics-toolkit/). It is generated using `mkdocs` and `mkdocs-material` and hosted on GitHub Pages.

## Acknowledgements

This repository was created to facilitate an internal project at [the RAI Institute](https://rai-inst.com/); it was developed by [Senthur Ayyappan](https://senthurayyappan.github.io/) and [Elliott Rouse](https://neurobionics.robotics.umich.edu/). We'd also like to acknowledge considerable support and guidance from Ben Bokser, [Daniel Gonzalez](https://dgonzrobotics.com/), [Surya Singh](https://scholar.google.com/citations?user=ZDzQGPQAAAAJ&hl=en&oi=sra), Sangbae Kim, and Stelian Coros at the AI Institute.

## Contributing

If you're interested in contributing to the project, please read the [contributing guidelines](https://github.com/neurobionics/onshape-robotics-toolkit/blob/main/CONTRIBUTING.md) to get started. All contributions are welcome!

## License

This project is licensed under the Apache 2.0 License. For more information, please refer to the [license](https://github.com/neurobionics/onshape-robotics-toolkit/blob/main/LICENSE) file.

## References

- [Onshape API Documentation](https://onshape-public.github.io/docs/)
- [Onshape API Glassworks Explorer](https://cad.onshape.com/glassworks/explorer/#/)
- [Onshape to Robot URDF Exporter](https://github.com/Rhoban/onshape-to-robot)
