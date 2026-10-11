"""Offline tests for fetching mass properties of rigid assemblies, including ones from linked documents."""

from __future__ import annotations

import asyncio
from threading import Lock
from typing import Any

import pytest

from onshape_robotics_toolkit.graph import KinematicGraph
from onshape_robotics_toolkit.models.assembly import Assembly, AssemblyInstance, RootAssembly
from onshape_robotics_toolkit.models.mass import MassProperties, PrincipalAxis
from onshape_robotics_toolkit.parse import CAD
from onshape_robotics_toolkit.robot import Robot

LINKED_DID = "aaaaaaaaaaaaaaaaaaaaaaaa"
LINKED_MICROVERSION = "bbbbbbbbbbbbbbbbbbbbbbbb"
LINKED_VERSION = "cccccccccccccccccccccccc"

MASS = MassProperties(
    volume=[1.0, 1.0, 1.0],
    mass=[2.0, 2.0, 2.0],
    centroid=[0.0, 0.0, 0.0],
    inertia=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
    principalInertia=[1.0, 1.0, 1.0],
    principalAxes=[
        PrincipalAxis(x=1.0, y=0.0, z=0.0),
        PrincipalAxis(x=0.0, y=1.0, z=0.0),
        PrincipalAxis(x=0.0, y=0.0, z=1.0),
    ],
)


class FakeClient:
    """Records every request made through the client surface used by CAD and Robot; never touches the network."""

    def __init__(self, assembly: Assembly, fail_mass: bool = False) -> None:
        self._assembly = assembly
        self._fail_mass = fail_mass
        self._lock = Lock()
        self.root_assembly_calls: list[dict[str, Any]] = []
        self.assembly_mass_calls: list[dict[str, Any]] = []
        self.part_mass_calls: list[dict[str, Any]] = []
        self.download_calls: list[dict[str, Any]] = []

    def get_root_assembly(self, **kwargs: Any) -> RootAssembly:
        with self._lock:
            self.root_assembly_calls.append(kwargs)
        # Occurrences of the requested subassembly, relative to it (as Onshape returns them)
        instance = next(
            i
            for i in self._assembly.rootAssembly.instances
            if isinstance(i, AssemblyInstance) and i.elementId == kwargs["eid"]
        )
        occurrences = [
            occ.model_copy(update={"path": occ.path[1:]})
            for occ in self._assembly.rootAssembly.occurrences
            if len(occ.path) > 1 and occ.path[0] == instance.id
        ]
        return RootAssembly.model_construct(occurrences=occurrences)

    def get_assembly_mass_properties(self, **kwargs: Any) -> MassProperties:
        with self._lock:
            self.assembly_mass_calls.append(kwargs)
        if self._fail_mass:
            raise ValueError(f"Assembly: {kwargs} does not have a mass property")
        return MASS

    def get_mass_property(self, **kwargs: Any) -> MassProperties:
        with self._lock:
            self.part_mass_calls.append(kwargs)
        if self._fail_mass:
            raise ValueError("Part does not have a mass property")
        return MASS

    def download_part_stl(self, **kwargs: Any) -> None:
        self.download_calls.append(kwargs)
        raise AssertionError("no mesh may be downloaded")

    def download_assembly_stl(self, **kwargs: Any) -> None:
        self.download_calls.append(kwargs)
        raise AssertionError("no mesh may be downloaded")


def _outer_instance(assembly: Assembly) -> AssemblyInstance:
    return next(i for i in assembly.rootAssembly.instances if isinstance(i, AssemblyInstance))


def _with_linked_outer_subassembly(assembly: Assembly) -> Assembly:
    """Copy of the fixture whose top-level subassembly lives in another (linked, versioned) document."""
    linked = assembly.model_copy(deep=True)
    instance = _outer_instance(linked)
    definition = next(s for s in linked.subAssemblies if s.uid == instance.uid)
    for model in (instance, definition):
        model.documentId = LINKED_DID
        model.documentMicroversion = LINKED_MICROVERSION
    instance.documentVersion = LINKED_VERSION
    return linked


def _outer_key(cad: CAD):
    return next(key for key, sub in cad.subassemblies.items() if not cad.is_nested_rigid_assembly(key))


def test_fixture_has_outer_and_nested_rigid_assemblies(cad_doc_depth_0: CAD) -> None:
    nested = [key for key in cad_doc_depth_0.subassemblies if cad_doc_depth_0.is_nested_rigid_assembly(key)]
    assert len(cad_doc_depth_0.subassemblies) == 3
    assert len(nested) == 2


@pytest.mark.parametrize("linked", [False, True])
def test_rigid_assembly_mass_uses_its_own_document_reference(assembly: Assembly, linked: bool) -> None:
    source = _with_linked_outer_subassembly(assembly) if linked else assembly
    cad = CAD.from_assembly(source, max_depth=0)
    client = FakeClient(source)

    failures = asyncio.run(cad.fetch_mass_properties_for_parts(client))  # type: ignore[arg-type]

    assert failures == {}
    assert len(client.assembly_mass_calls) == 1, "only the outermost rigid assembly is fetched"
    call = client.assembly_mass_calls[0]
    instance = cad.instances[_outer_key(cad)]
    assert call == {
        "did": instance.documentId,
        "wtype": "m",
        "wid": instance.documentMicroversion,
        "eid": instance.elementId,
    }
    if linked:
        assert call["did"] == LINKED_DID
        assert call["wid"] == LINKED_MICROVERSION
    assert call["wid"] != cad.workspace_id


def test_linked_rigid_assembly_has_no_root_workspace_id(assembly: Assembly) -> None:
    linked = CAD.from_assembly(_with_linked_outer_subassembly(assembly), max_depth=0)
    same_doc = CAD.from_assembly(assembly, max_depth=0)

    assert linked.parts[_outer_key(linked)].rigidAssemblyWorkspaceId is None
    assert same_doc.parts[_outer_key(same_doc)].rigidAssemblyWorkspaceId == same_doc.workspace_id


def test_mesh_reference_is_unchanged(assembly: Assembly) -> None:
    """Mesh downloads keep using the version (linked) or the root workspace (same document)."""
    from onshape_robotics_toolkit.robot import get_robot_link

    for source, expected in (
        (_with_linked_outer_subassembly(assembly), ("v", LINKED_VERSION)),
        (assembly, ("w", None)),
    ):
        cad = CAD.from_assembly(source, max_depth=0)
        _, _, asset = get_robot_link("link", cad.parts[_outer_key(cad)], client=None, mate=None)  # type: ignore[arg-type]
        assert asset.wtype == expected[0]
        assert asset.wid == (expected[1] or cad.workspace_id)


def test_occurrence_fetch_skips_nested_and_brings_the_mass(assembly: Assembly) -> None:
    """Parsing with a client fetches occurrences and mass once per outermost rigid assembly, nothing else."""
    source = _with_linked_outer_subassembly(assembly)
    client = FakeClient(source)

    cad = CAD.from_assembly(source, max_depth=0, client=client, fetch_mate_properties=False)  # type: ignore[arg-type]

    assert len(client.root_assembly_calls) == 1
    call = client.root_assembly_calls[0]
    assert (call["did"], call["wtype"], call["wid"]) == (LINKED_DID, "m", LINKED_MICROVERSION)
    assert call["with_meta_data"] is False
    assert len(client.assembly_mass_calls) == 1
    assert cad.parts[_outer_key(cad)].MassProperty == MASS

    # Mass came with the occurrences, so the mass step needs no further request for it
    asyncio.run(cad.fetch_mass_properties_for_parts(client, keys=[_outer_key(cad)]))  # type: ignore[arg-type]
    assert len(client.assembly_mass_calls) == 1

    estimate = cad.estimate_api_calls(fetch_mate_properties=False)
    assert estimate["subassemblies"] == 2
    free_parts = sum(1 for p in cad.parts.values() if p.rigidAssemblyToPartTF is None and not p.isRigidAssembly)
    assert estimate["mass_properties"] == free_parts, "the rigid assembly's mass is not requested again"


def test_fetch_only_given_keys(cad_doc_depth_0: CAD, assembly: Assembly) -> None:
    client = FakeClient(assembly)
    key = _outer_key(cad_doc_depth_0)

    asyncio.run(cad_doc_depth_0.fetch_mass_properties_for_parts(client, keys=[key]))  # type: ignore[arg-type]

    assert len(client.assembly_mass_calls) == 1
    assert client.part_mass_calls == []
    assert cad_doc_depth_0.parts[key].MassProperty == MASS


def _graph_with_rigid_assembly(cad: CAD) -> KinematicGraph:
    graph = KinematicGraph.from_cad(cad)
    assert any(cad.parts[key].isRigidAssembly for key in graph.nodes)
    return graph


def test_from_graph_raises_before_download_when_mass_is_missing(assembly: Assembly) -> None:
    cad = CAD.from_assembly(_with_linked_outer_subassembly(assembly), max_depth=0)
    graph = _graph_with_rigid_assembly(cad)
    client = FakeClient(assembly, fail_mass=True)

    with pytest.raises(ValueError, match="Mass properties missing") as excinfo:
        Robot.from_graph(graph, client=client, name="robot", fetch_mass_properties=True)  # type: ignore[arg-type]

    message = str(excinfo.value)
    assert str(_outer_key(cad)) in message
    assert LINKED_DID in message, "the error names the request that was tried"
    assert client.download_calls == []


def test_from_graph_allow_missing_mass_falls_back_to_defaults(assembly: Assembly) -> None:
    cad = CAD.from_assembly(assembly, max_depth=0)
    graph = _graph_with_rigid_assembly(cad)
    client = FakeClient(assembly, fail_mass=True)

    robot = Robot.from_graph(
        graph,
        client=client,  # type: ignore[arg-type]
        name="robot",
        fetch_mass_properties=True,
        allow_missing_mass=True,
    )

    assert len(robot.nodes) >= len(graph.nodes)
    assert client.download_calls == []


def test_from_graph_fetches_only_links(assembly: Assembly) -> None:
    cad = CAD.from_assembly(assembly, max_depth=0)
    graph = _graph_with_rigid_assembly(cad)
    client = FakeClient(assembly)

    Robot.from_graph(graph, client=client, name="robot", fetch_mass_properties=True)  # type: ignore[arg-type]

    requested = len(client.assembly_mass_calls) + len(client.part_mass_calls)
    assert requested == len(graph.nodes)
    assert all(graph.nodes[key]["data"].MassProperty is not None for key in graph.nodes)
