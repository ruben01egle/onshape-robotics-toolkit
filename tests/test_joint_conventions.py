"""Offline tests for root-mate frames inside rigid assemblies and the joint-limit sign convention."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from onshape_robotics_toolkit.graph import KinematicGraph, resolve_root_mate
from onshape_robotics_toolkit.models.assembly import MatedCS, MatedEntity, MateFeatureData, MateType, Part
from onshape_robotics_toolkit.parse import CAD, PathKey
from onshape_robotics_toolkit.robot import get_robot_joint, get_robot_link


def _tf(xyz: tuple[float, float, float], rotvec: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> np.ndarray:
    tf = np.eye(4)
    tf[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
    tf[:3, 3] = xyz
    return tf


def _mate(mate_type: MateType, limits=None, reversed_: bool = False) -> MateFeatureData:
    return MateFeatureData(
        matedEntities=[
            MatedEntity(matedOccurrence=["parent"], matedCS=MatedCS.from_tf(np.eye(4))),
            MatedEntity(matedOccurrence=["child"], matedCS=MatedCS.from_tf(np.eye(4))),
        ],
        mateType=mate_type,
        name="axis2",
        id="axis2",
        limits=limits,
        entitiesReversed=reversed_,
    )


# --- Root mate on a part inside a rigid subassembly ---------------------------------------------

RIGID_KEY = PathKey(("base",), ("Base_1",))
PART_KEY = PathKey(("base", "plate"), ("Base_1", "Plate_1"))
# Part pose in the rigid assembly's frame, and the root connector in the part's frame (z = -86 mm)
RIGID_TO_PART = _tf((0.01, -0.02, 0.086), (0.0, 0.0, np.pi / 2))
PART_TO_CONNECTOR = _tf((0.0, 0.0, -0.086))


def _root_mate_cad(rigid_to_part=RIGID_TO_PART) -> SimpleNamespace:
    root_mate = MateFeatureData(
        matedEntities=[
            MatedEntity(matedOccurrence=list(PART_KEY.path), matedCS=MatedCS.from_tf(PART_TO_CONNECTOR)),
            MatedEntity(matedOccurrence=[], matedCS=MatedCS.from_tf(np.eye(4))),
        ],
        mateType=MateType.FASTENED,
        name="root_mate",
        id="root_mate",
    )
    part = Part.model_construct(
        rigidAssemblyKey=RIGID_KEY,
        rigidAssemblyToPartTF=None if rigid_to_part is None else MatedCS.from_tf(rigid_to_part),
    )
    keys = {PART_KEY.path: PART_KEY}
    return SimpleNamespace(
        root_mate_candidates={"root_mate": root_mate},
        mates={},
        parts={PART_KEY: part},
        get_path_key=lambda path: keys.get(tuple(path)),
    )


def test_root_mate_inside_rigid_assembly_is_expressed_in_the_assembly_frame() -> None:
    cad = _root_mate_cad()

    root_key, mate = resolve_root_mate(cad, "root_mate")  # type: ignore[arg-type]

    assert root_key == RIGID_KEY
    assert mate.matedEntities[0].matedOccurrence == []
    assert mate.matedEntities[-1].matedOccurrence == list(RIGID_KEY.path)
    assert np.allclose(mate.matedEntities[-1].matedCS.to_tf, RIGID_TO_PART @ PART_TO_CONNECTOR)
    # The CAD's own candidate is left untouched
    candidate = cad.root_mate_candidates["root_mate"].matedEntities[0]
    assert candidate.matedOccurrence == list(PART_KEY.path)
    assert np.allclose(candidate.matedCS.to_tf, PART_TO_CONNECTOR)

    # The root link's frame (in the rigid assembly's mesh frame) is that composed connector
    rigid_part = Part.model_construct(
        isRigidAssembly=True,
        documentId="d" * 24,
        elementId="e" * 24,
        partId="e" * 24,
        documentMicroversion="m" * 24,
        documentVersion="v" * 24,
        rigidAssemblyWorkspaceId=None,
        MassProperty=None,
        worldToPartTF=None,
    )
    _, world_to_link, _ = get_robot_link("base", rigid_part, client=None, mate=mate)  # type: ignore[arg-type]
    assert np.allclose(np.linalg.inv(world_to_link), RIGID_TO_PART @ PART_TO_CONNECTOR)
    # The connector sits at the assembly's z = 0 here, so the base mesh is not shifted down by 86 mm
    assert np.isclose(np.asarray(world_to_link)[2, 3], 0.0)


def test_root_mate_inside_rigid_assembly_without_pose_raises() -> None:
    with pytest.raises(ValueError, match="pose in that assembly is unknown"):
        resolve_root_mate(_root_mate_cad(rigid_to_part=None), "root_mate")  # type: ignore[arg-type]


# --- Joint limits ---------------------------------------------------------------------------------


@pytest.mark.parametrize("mate_type", [MateType.REVOLUTE, MateType.SLIDER, MateType.CYLINDRICAL])
@pytest.mark.parametrize(("reversed_", "expected"), [(False, (-2.356, 0.349)), (True, (-0.349, 2.356))])
def test_api_limits_follow_mate_direction(mate_type, reversed_, expected) -> None:
    mate = _mate(mate_type, limits={"min": -2.356, "max": 0.349}, reversed_=reversed_)

    joints, _ = get_robot_joint(
        PathKey(("parent",), ("parent",)), PathKey(("child",), ("child",)), mate, np.eye(4), set()
    )

    joint = next(iter(joints.values()))
    assert (joint.limits.lower, joint.limits.upper) == pytest.approx(expected)
    assert joint.axis.xyz == (0.0, 0.0, 1.0)


@pytest.mark.parametrize(("reversed_", "expected"), [(False, (-1.0, 0.5)), (True, (-0.5, 1.0))])
def test_config_limits_follow_mate_direction(monkeypatch, reversed_, expected) -> None:
    from onshape_robotics_toolkit import robot as robot_module

    monkeypatch.setattr(robot_module, "resolve_mate_limits", lambda name: {"min": -1.0, "max": 0.5})
    mate = _mate(MateType.REVOLUTE, limits={"min": -2.356, "max": 0.349}, reversed_=reversed_)

    joints, _ = get_robot_joint(
        PathKey(("parent",), ("parent",)), PathKey(("child",), ("child",)), mate, np.eye(4), set()
    )

    joint = next(iter(joints.values()))
    assert (joint.limits.lower, joint.limits.upper) == pytest.approx(expected)


def test_graph_flags_exactly_the_reversed_mates(cad_doc: CAD) -> None:
    graph = KinematicGraph.from_cad(cad_doc)
    original = graph._remap_mates(cad_doc)

    flags = {}
    for parent, child, data in graph.edges(data=True):
        mate = data["data"]
        reversed_in_graph = (parent, child) not in original and (child, parent) in original
        assert mate.entitiesReversed == reversed_in_graph
        flags[(parent, child)] = mate.entitiesReversed
        # The source mates are never marked
        source = original.get((parent, child)) or original[(child, parent)]
        assert source.entitiesReversed is False

    assert any(flags.values()) and not all(flags.values()), "fixture should have both kinds of mates"
