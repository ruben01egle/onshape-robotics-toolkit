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
