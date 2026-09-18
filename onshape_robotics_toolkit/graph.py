"""
This module contains functions and classes to create and manipulate kinematic graphs from Onshape assembly data.

The main class is KinematicGraph, which uses the PathKey-based CAD system to build a directed graph
representing the kinematic structure of a robot assembly. The graph nodes are parts involved in mates,
and edges represent mate relationships.

Classes:
    KinematicGraph: Build and navigate kinematic graph from CAD assembly

Functions:
    plot_graph: Visualize graphs using matplotlib
    get_root_node: Get root node of directed graph
    convert_to_digraph: Convert undirected graph to directed with root detection
    get_topological_order: Calculate topological ordering
    remove_unconnected_subgraphs: Remove disconnected components from graph
"""

import copy
import random
from typing import Optional, Union

import matplotlib.pyplot as plt
import networkx as nx
from loguru import logger

from onshape_robotics_toolkit.config import record_kinematics_config
from onshape_robotics_toolkit.models.assembly import MatedCS, MateFeatureData, MateType
from onshape_robotics_toolkit.parse import CAD, CHILD, PARENT, PathKey
from onshape_robotics_toolkit.utilities.helpers import get_sanitized_name


def convert_to_digraph(graph: nx.Graph, user_defined_root: Optional[PathKey] = None) -> tuple[nx.DiGraph, PathKey]:
    """
    Convert a graph to a directed graph and calculate the root node using closeness centrality.

    Args:
        graph: The graph to convert.
        user_defined_root: The node to use as the root node.

    Returns:
        The directed graph and the root node of the graph, calculated using closeness centrality.

    Examples:
        >>> graph = nx.Graph()
        >>> convert_to_digraph(graph)
        (digraph, root_node)
    """

    centrality = nx.closeness_centrality(graph)
    root_node = user_defined_root if user_defined_root else max(centrality, key=lambda x: centrality[x])

    # Create BFS tree from root (this loses edge data!)
    bfs_graph = nx.bfs_tree(graph, root_node)
    di_graph = nx.DiGraph(bfs_graph)

    # Restore edge data for BFS tree edges from original graph
    for u, v in list(di_graph.edges()):
        if graph.has_edge(u, v):
            # Copy edge data from original undirected graph
            di_graph[u][v].update(graph[u][v])
        elif graph.has_edge(v, u):
            # Edge might be reversed in undirected graph
            di_graph[u][v].update(graph[v][u])

    # Add back any edges not in BFS tree (loops, etc.)
    for u, v, data in graph.edges(data=True):
        if not di_graph.has_edge(u, v) and not di_graph.has_edge(v, u):
            # Decide which direction to keep based on centrality
            if centrality[u] > centrality[v]:
                di_graph.add_edge(u, v, **data)
            else:
                di_graph.add_edge(v, u, **data)

    return di_graph, root_node


def resolve_root_mate(cad: CAD, root_mate_name: str) -> tuple[PathKey, MateFeatureData]:
    """
    Resolve the kinematic root from a named mate-to-origin, instead of a `.fixed` occurrence.

    In Onshape, create a mate connector from the top-level assembly's own Origin (no dummy
    fixed part needed) and FASTEN it to a mate connector placed inside the intended root part
    or subassembly. That mate's assembly-origin side has an empty `matedOccurrence`; the other
    side's occurrence identifies the root part.

    Args:
        cad: CAD assembly with PathKey-based registries (already fetched).
        root_mate_name: Name of the top-level FASTENED mate-to-origin.

    Returns:
        A tuple of (root PathKey, the mate's MateFeatureData with matedEntities ordered as
        [assembly-origin side, root-part side], so `matedEntities[-1].matedCS` gives the
        root link's frame exactly like any other joint's child-side mate connector).

    Raises:
        ValueError: If the mate can't be found or resolved, identifying which check failed.
            This never falls back to auto-detection - a wrong root would waste every
            subsequent API call computing the wrong robot.
    """
    mate = cad.root_mate_candidates.get(root_mate_name)
    if mate is None:
        if any(existing_mate.name == root_mate_name for existing_mate in cad.mates.values()):
            raise ValueError(
                f"root_mate_name='{root_mate_name}' matches an existing mate, but it has an "
                "occurrence on both sides (not a mate-to-assembly-origin). Create a mate "
                "connector from the top-level assembly's own Origin and FASTEN it to the "
                "intended root part/subassembly instead."
            )
        raise ValueError(
            f"root_mate_name='{root_mate_name}' does not match any top-level mate-to-origin "
            "candidate. Check the mate name and make sure it is a top-level (not subassembly) "
            "FASTENED mate between the assembly Origin and the intended root part."
        )

    if mate.mateType != MateType.FASTENED:
        raise ValueError(
            f"root_mate_name='{root_mate_name}' resolves to a mate of type "
            f"'{mate.mateType}', not FASTENED. A mate-to-origin root anchor must be FASTENED."
        )

    empty_entities = [entity for entity in mate.matedEntities if not entity.matedOccurrence]
    nonempty_entities = [entity for entity in mate.matedEntities if entity.matedOccurrence]
    if len(empty_entities) != 1 or len(nonempty_entities) != 1:
        raise ValueError(
            f"root_mate_name='{root_mate_name}' does not have exactly one assembly-origin side "
            f"(empty matedOccurrence) and one part-occurrence side: "
            f"{len(empty_entities)} empty, {len(nonempty_entities)} non-empty."
        )

    origin_entity = empty_entities[0]
    other_entity = nonempty_entities[0]

    other_path = tuple(other_entity.matedOccurrence)
    root_key = cad.get_path_key(other_path)
    if root_key is None:
        raise ValueError(
            f"root_mate_name='{root_mate_name}' occurrence path {other_path} does not resolve "
            "to any known part in this assembly."
        )

    # NOTE: mirrors the rigid-assembly remap in KinematicGraph._remap_mates - if the resolved
    # part belongs to a rigid subassembly, the graph's nodes are keyed by the rigid assembly
    # root, not the individual part.
    part = cad.parts.get(root_key)
    if part is not None and part.rigidAssemblyKey is not None:
        root_key = part.rigidAssemblyKey

    resolved_mate = copy.deepcopy(mate)
    resolved_mate.matedEntities = [origin_entity, other_entity]

    return root_key, resolved_mate


def create_graph(
    part_keys: set[PathKey],
    mates: dict[tuple[PathKey, PathKey], MateFeatureData],
) -> nx.Graph:
    def _add_nodes(part_keys: set[PathKey], graph: nx.Graph) -> None:
        """
        Add nodes to graph for parts involved in mates.

        With eager parts population, all valid mate targets are guaranteed to be
        in cad.parts. We only need to filter hidden occurrences.

        Args:
            part_keys: Set of PathKeys to add as nodes
            parts: Dictionary of all parts in the CAD assembly
            graph: The graph to add nodes to
        """
        logger.debug(f"Processing {len(part_keys)} involved parts")
        for part_key in part_keys:
            # Check if instance is suppressed
            # instance = self.cad.instances.get(part_key)
            # if instance and instance.suppressed:
            #     # TODO: Should this happen here or within the CAD class?
            #     # Skipping parts here is problematic since the mate registry still
            #     # refers to them, leading to dangling edges.
            #     logger.debug(f"Skipping suppressed part: {part_key}")
            #     skipped_hidden += 1
            #     continue
            graph.add_node(
                part_key,
            )

        logger.debug(f"Added {len(graph.nodes)} nodes to graph")

    def _add_edges(mates: dict[tuple[PathKey, PathKey], MateFeatureData], graph: nx.Graph) -> None:
        """
        Add edges to graph from mate relationships.

        Args:
            mates: Dictionary of mate relationships
            graph: The graph to add edges to
        """
        # NOTE: since we are adding edges to an undirected graph,
        # the order of the parent-child keys are not preserved within
        # the graph structure, i.e. an edge (u, v) could become (v, u)
        # after the fact since the graph is undirected.
        for (parent_key, child_key), _ in mates.items():
            if parent_key in graph.nodes and child_key in graph.nodes:
                graph.add_edge(
                    parent_key,
                    child_key,
                )
                logger.debug(f"Added edge: {parent_key} -> {child_key}")
            else:
                logger.debug(f"Skipping edge {parent_key} -> {child_key} (nodes not in graph)")

        logger.debug(f"Added {len(graph.edges)} edges to graph")

    graph: nx.Graph = nx.Graph()
    _add_nodes(part_keys, graph)
    _add_edges(mates, graph)

    return graph


def _print_graph_tree(graph: nx.Graph) -> None:
    """Print a text-based tree representation of the graph structure."""
    if not graph.nodes:
        logger.info("  (empty graph)")
        return

    # Show connected components
    components = list(nx.connected_components(graph))
    for i, component in enumerate(components):
        logger.info(f"  Root {i + 1} ({len(component)} nodes):")
        for j, node in enumerate(sorted(component)):
            prefix = "    ├── " if j < len(component) - 1 else "    └── "
            neighbors = list(graph.neighbors(node))
            neighbor_info = f" -> {len(neighbors)} connections" if neighbors else ""
            logger.info(f"{prefix}{node}{neighbor_info}")
        if i < len(components) - 1:
            logger.info("")


def remove_disconnected_subgraphs(graph: nx.Graph) -> nx.Graph:
    """
    Remove unconnected subgraphs from the graph.

    Args:
        graph: The graph to remove unconnected subgraphs from.

    Returns:
        The main connected subgraph of the graph, which is the largest connected subgraph.
    """
    # Handle empty graph case (e.g., assemblies with only mate groups)
    if len(graph.nodes) == 0:
        logger.debug("Graph is empty (no nodes) - this may indicate an assembly with only mate groups")
        return graph

    if not nx.is_connected(graph):
        logger.warning("Graph has one or more unconnected subgraphs")

        # Show tree visualization of original graph
        logger.info("Original graph structure:")
        _print_graph_tree(graph)

        sub_graphs = list(nx.connected_components(graph))
        main_graph_nodes = max(sub_graphs, key=len)
        main_graph = graph.subgraph(main_graph_nodes).copy()

        # Show tree visualization of reduced graph
        logger.info("Reduced graph structure:")
        _print_graph_tree(main_graph)

        logger.warning(f"Reduced graph nodes from {len(graph.nodes)} to {len(main_graph.nodes)}")
        logger.warning(f"Reduced graph edges from {len(graph.edges)} to {len(main_graph.edges)}")
        return main_graph
    return graph


class KinematicGraph(nx.DiGraph):
    """
    kinematic graph representation of an assembly using PathKey-based system.

    This class creates a directed graph from CAD assembly data where:
    - Nodes: Parts involved in mates (PathKey identifiers)
    - Edges: Mate relationships between parts
    - Root: Determined by closeness centrality or user-defined fixed part

    The tree supports:
    - Topological ordering for kinematic chains
    - Root node detection via centrality or user preference
    - Disconnected subgraph removal
    - Visualization

    Attributes:
        cad: CAD assembly data with PathKey-based registries
        root_node: PathKey of the root node in the kinematic graph
        topological_order: Ordered sequence of nodes from root to leaves
        root_mate: When the root was resolved via `root_mate_name`, the mate's
            MateFeatureData (matedEntities ordered [origin side, root-part side]).
            None when the root came from the legacy `.fixed`/centrality path.
    """

    # TODO: make sure we are not mutating classes and instead creating copies
    # refactor any method that mutates data in place and creates a destructive change

    def __init__(self, cad: CAD):
        """
        Initialize kinematic graph from CAD data.

        Note: The preferred way to create a KinematicGraph is via the `from_cad()`
        classmethod, which makes the construction more explicit and allows
        configuring root node detection.

        Args:
            cad: CAD assembly with PathKey-based registries

        Examples:
            >>> # Preferred (explicit):
            >>> graph = KinematicGraph.from_cad(cad, use_user_defined_root=True)

            >>> # Also works (backward compatible):
            >>> graph = KinematicGraph(cad)
        """
        self.cad = cad
        self.root: Optional[PathKey] = None
        self.root_mate: Optional[MateFeatureData] = None

        super().__init__()

    @classmethod
    def from_cad(
        cls,
        cad: CAD,
        use_user_defined_root: bool = True,
        root_mate_name: Optional[str] = None,
    ) -> "KinematicGraph":
        """
        Create and build kinematic graph from CAD assembly.

        This is the recommended way to create a KinematicGraph. It constructs
        the graph by processing mates, validating PathKeys, and determining
        the kinematic structure.

        Args:
            cad: CAD assembly with PathKey-based registries
            use_user_defined_root: Whether to use user-marked fixed part as root.
                Ignored when `root_mate_name` is given.
            root_mate_name: Name of a top-level FASTENED mate between the assembly's
                own Origin and the intended root part/subassembly (a "mate-to-origin").
                Supersedes needing a dummy fixed part just to anchor the root. When
                given, resolution is strict: if the mate can't be found or resolved,
                this raises immediately instead of silently falling back to
                `use_user_defined_root`/centrality-based auto-detection - continuing
                with a wrong root would waste every subsequent API call computing the
                wrong robot.

        Returns:
            Fully constructed KinematicGraph with nodes, edges, and root

        Examples:
            >>> cad = CAD.from_assembly(assembly, max_depth=1)
            >>> graph = KinematicGraph.from_cad(cad, use_user_defined_root=True)
            >>> print(f"Root: {graph.root_node}")
            >>> print(f"Nodes: {len(graph.graph.nodes)}")

            >>> # Or, with a mate-to-origin instead of a dummy fixed part:
            >>> graph = KinematicGraph.from_cad(cad, root_mate_name="root_mate")
        """
        kinematic_graph = cls(cad=cad)

        preset_root: Optional[PathKey] = None
        if root_mate_name is not None:
            preset_root, kinematic_graph.root_mate = resolve_root_mate(cad, root_mate_name)

        kinematic_graph._build_graph(use_user_defined_root, preset_root=preset_root)

        record_kinematics_config(use_user_defined_root=use_user_defined_root)

        return kinematic_graph

    def _build_graph(self, use_user_defined_root: bool, preset_root: Optional[PathKey] = None) -> None:
        """
        Build kinematic graph from CAD assembly data.

        Process:
        1. Collect all mates from root and subassemblies
        2. Validate and filter mates (remove invalid PathKeys)
        3. Get parts involved in valid mates
        4. Add nodes for involved parts (with metadata)
        5. Add edges from mate relationships

        Args:
            use_user_defined_root: Whether to use user-defined fixed part as root
            preset_root: Root PathKey already resolved (e.g. via a named mate-to-origin),
                bypassing `.fixed`/centrality-based root detection entirely.
        """
        # remap the mates to switch out any parts that belong to rigid subassemblies
        remapped_mates = self._remap_mates(self.cad)
        involved_parts = self._get_parts_involved_in_mates(remapped_mates)

        raw_graph = create_graph(
            part_keys=involved_parts,
            mates=remapped_mates,
        )

        self._process_graph(raw_graph, involved_parts, remapped_mates, use_user_defined_root, preset_root)

        if len(self.nodes) == 0:
            logger.warning("KinematicGraph is empty - no valid parts found in mates")
            return

        logger.info(
            f"KinematicGraph processed: {len(self.nodes)} nodes, "
            f"{len(self.edges)} edges with root node: {self.root}"
        )

    def _process_graph(
        self,
        raw_graph: nx.Graph,
        parts: set[PathKey],
        mates: dict[tuple[PathKey, PathKey], MateFeatureData],
        use_user_defined_root: bool,
        preset_root: Optional[PathKey] = None,
    ) -> None:
        """
        Process the graph:
            1. Remove disconnected subgraphs
            2. Convert to directed graph with root detection
            3. Calculate topological order
        """
        # remove disconnected subgraphs
        graph = remove_disconnected_subgraphs(raw_graph)

        # Handle empty graph case (assemblies with only mate groups and no fixed/rigid parts)
        if len(graph.nodes) == 0:
            if preset_root is not None:
                raise ValueError(
                    f"root_mate_name resolved to part {preset_root}, but the kinematic graph has "
                    "no nodes at all (no mates found). Cannot use it as a root."
                )
            logger.warning(
                "Graph has no nodes - assembly contains only mate groups with no rigid assemblies or fixed parts. "
                "Cannot create kinematic graph."
            )
            return

        # Handle single-node graph case (e.g., single rigid assembly from mate groups)
        if len(graph.nodes) == 1:
            logger.info("Graph has single node - this is a fully rigid assembly (one link, no joints)")
            single_node = next(iter(graph.nodes))
            if preset_root is not None and preset_root != single_node:
                raise ValueError(
                    f"root_mate_name resolved to part {preset_root}, but the kinematic graph's "
                    f"only node is {single_node}. Check the mate name and target part."
                )
            self.root = single_node
            part = self.cad.parts[single_node]
            self.add_node(single_node, data=part)
            return

        if preset_root is not None:
            if preset_root not in graph.nodes:
                raise ValueError(
                    f"root_mate_name resolved to part {preset_root}, which is not part of the "
                    "connected kinematic graph (it is not involved in any mate). Check the "
                    "mate and the Base part it targets."
                )
            self.root = preset_root
            logger.debug(f"Using mate-resolved root: {preset_root}")
        else:
            self._find_root_node(
                graph=graph,
                parts=parts,
                use_user_defined_root=use_user_defined_root,
            )

        bfs_graph = nx.bfs_tree(graph, self.root)
        # NOTE: add all nodes in the BFS order
        for node in bfs_graph.nodes:
            part = self.cad.parts[node]
            self.add_node(
                node,
                data=part,
            )

        for u, v in list(bfs_graph.edges()):
            # NOTE: if raw graph has edge u->v, then mates also has (u,v) key, right?
            # NOOOO, since the graph is undirected, the edge could be (v,u) instead
            # even though we added the edge as (u,v), hence we use the mates dict
            # instead to check the original parent->child order
            if (u, v) in mates:
                mate = copy.deepcopy(mates[(u, v)])
                self.add_edge(
                    u,
                    v,
                    data=mate,
                )
            elif (v, u) in mates:
                # the mate parent->child order has flipped
                mate = copy.deepcopy(mates[(v, u)])
                mate.matedEntities.reverse()
                # NOTE: we are mutating the mate data here, but
                # preserving the BFS tree parent->child order
                self.add_edge(
                    u,
                    v,
                    data=mate,
                )

        # Add back any edges not in BFS tree (loops, etc.)
        for u, v in mates:
            if not bfs_graph.has_edge(u, v) and not bfs_graph.has_edge(v, u):
                # preserve the original parent->child order
                self.add_edge(
                    u,
                    v,
                    data=mates[(u, v)],
                )

    def _remap_mates(self, cad: CAD) -> dict[tuple[PathKey, PathKey], MateFeatureData]:
        """
        Remap mates to replace parts that belong to rigid subassemblies with the rigid assembly part

        Args:
            cad (CAD): The CAD assembly generated from Onshape data

        Returns:
            dict[tuple[PathKey, PathKey], MateFeatureData]: A mapping of original mate paths
            (w/o assembly keys) to their remapped counterparts
        """

        def remap_mate(key: PathKey, index: int, mate: MateFeatureData) -> PathKey:
            """
            Return the rigid assembly root key if the part is inside a rigid assembly,
            otherwise return the original key.
            """
            r_key: PathKey = key
            part = cad.parts[key]
            if part.rigidAssemblyKey is not None:
                r_key = part.rigidAssemblyKey
                # NOTE: this is where we remap the matedOccurrence as well
                mated_part_entity = mate.matedEntities[index]
                mated_part_entity.matedOccurrence = list(r_key.path)

                if part.rigidAssemblyToPartTF is None:
                    logger.warning(
                        f"Part {key} belongs to rigid assembly {r_key} but has no rigidAssemblyToPartTF set. \n"
                        "This will result in malformed joints that have refer to parts within rigid assemblies."
                    )
                    return r_key

                # MatedCS remapping from part->mate to rigid_root->mate
                mated_part_entity.matedCS = MatedCS.from_tf(
                    part.rigidAssemblyToPartTF.to_tf @ mated_part_entity.matedCS.to_tf
                )

            return r_key

        remapped_mates: dict[tuple[PathKey, PathKey], MateFeatureData] = {}
        # CAD's mates are already filtered and validated, they only include mates that
        # need to be processed for robot generation
        for (_, *entities), mate in cad.mates.items():
            _mate_data = copy.deepcopy(mate)
            remapped_keys = []

            for i, key in enumerate(entities):
                # NOTE: mate data's matedEntities have matedOccurrences that need to
                # be remapped as well in addition to the keys
                remapped_key = remap_mate(key, i, _mate_data)
                remapped_keys.append(remapped_key)

            remapped_mate_key: tuple[PathKey, PathKey] = tuple(remapped_keys)  # type: ignore[assignment]
            if remapped_mate_key in remapped_mates:
                logger.warning(
                    "Duplicate mate detected after remapping: %s -> %s. "
                    "This can happen if multiple parts in a rigid assembly are mated to the same part. "
                    "Only the first mate will be kept.",
                    remapped_mate_key[PARENT],
                    remapped_mate_key[CHILD],
                )
                continue

            remapped_mates[remapped_mate_key] = _mate_data
        return remapped_mates

    def _is_root_assembly_rigid(self) -> bool:
        """
        Check if the root assembly should be treated as rigid.

        Root assembly is rigid if it has only mate groups (no regular mates).
        This is indicated by having 0 mates in cad.mates at the root level.

        Returns:
            True if root assembly has only mate groups, False otherwise
        """
        # Check if root assembly has any regular mates (not from subassemblies)
        root_mates = [
            mate
            for (assembly_key, _, _), mate in self.cad.mates.items()
            if assembly_key is None  # Root level mates
        ]

        has_root_mates = len(root_mates) > 0

        if not has_root_mates:
            logger.debug("Root assembly has no regular mates - checking features")
            # No mates at root level means root is rigid (only mate groups)
            return True

        return False

    def _get_parts_involved_in_mates(self, mates: dict[tuple[PathKey, PathKey], MateFeatureData]) -> set[PathKey]:
        """
        Extract all part PathKeys that should be nodes in the kinematic graph.

        This includes:
        1. Parts involved in mates (normal case)
        2. When root is rigid: single node representing entire assembly
        3. When root has mates: rigid assemblies and root-level parts as separate nodes

        Args:
            mates: Dictionary of mate relationships

        Returns:
            Set of PathKeys for parts that should be graph nodes
        """
        involved_parts: set[PathKey] = set()

        # Add parts involved in mates
        for parent_key, child_key in mates:
            involved_parts.add(parent_key)
            involved_parts.add(child_key)

        # If no mates exist, check if root assembly is rigid
        if len(mates) == 0:
            root_is_rigid = self._is_root_assembly_rigid()

            if root_is_rigid:
                logger.info(
                    "Root assembly is rigid (only mate groups) - entire assembly will be one node. "
                    "All parts and subassemblies merged into single rigid body."
                )
                # Pick one representative node for the entire rigid assembly
                # Priority: fixed parts > rigid subassemblies > first root part
                for part_key, _ in self.cad.parts.items():
                    occurrence = self.cad.occurrences.get(part_key)
                    if occurrence and occurrence.fixed:
                        involved_parts.add(part_key)
                        logger.debug(f"Using fixed part as root for rigid assembly: {part_key}")
                        break

                if len(involved_parts) == 0:
                    for part_key, part in self.cad.parts.items():
                        if part.isRigidAssembly:
                            involved_parts.add(part_key)
                            logger.debug(f"Using rigid subassembly as root: {part_key}")
                            break

                if len(involved_parts) == 0:
                    for part_key, _ in self.cad.parts.items():
                        if part_key.depth == 0:
                            involved_parts.add(part_key)
                            logger.debug(f"Using first root part as root: {part_key}")
                            break
            else:
                logger.debug("Root not rigid - adding parts as separate nodes")
                for part_key, part in self.cad.parts.items():
                    if part.isRigidAssembly:
                        involved_parts.add(part_key)
                        logger.debug(f"Adding rigid assembly as node: {part_key}")
                    elif part_key.depth == 0:
                        involved_parts.add(part_key)
                        logger.debug(f"Adding root-level part as node: {part_key}")
                    else:
                        occurrence = self.cad.occurrences.get(part_key)
                        if occurrence and occurrence.fixed:
                            involved_parts.add(part_key)
                            logger.debug(f"Adding fixed part as node: {part_key}")

        logger.debug(f"Found {len(involved_parts)} parts to include in graph")
        return involved_parts

    def _find_root_node(self, graph: nx.Graph, parts: set[PathKey], use_user_defined_root: bool) -> None:
        """
        Find user-defined root part (marked as fixed in Onshape).

        Args:
            involved_parts: Set of parts to search within

        Returns:
            PathKey of fixed part, or None if not found
        """
        root = None
        if use_user_defined_root:
            for part_key in parts:
                occurrence = self.cad.occurrences.get(part_key)
                if occurrence and occurrence.fixed:
                    logger.debug(f"Found user-defined root: {part_key}")
                    root = part_key
                    self.root = root
                    break

            if root is None:
                logger.warning("No user-defined root part found (marked as fixed in Onshape), auto-detecting root")
                self._find_root_node(graph, parts, use_user_defined_root=False)
        else:
            centrality = nx.closeness_centrality(graph)
            root = max(centrality, key=lambda x: centrality[x])
            if root:
                self.root = root
                logger.debug(f"Auto-detected root node: {root}")
            else:
                logger.warning("Could not determine root node via topological sort")

    def show(self, file_name: Optional[str] = None, graph: Optional[Union[nx.Graph, nx.DiGraph]] = None) -> None:
        """
        Visualize the kinematic graph with part names as labels instead of PathKey IDs.

        Creates a more readable visualization by mapping PathKeys to their corresponding
        part or assembly names from the CAD instance registry.

        Args:
            file_name: Optional filename to save visualization. If None, displays interactively.
            graph: Optional graph to visualize. If None, uses the current graph.

        Examples:
            >>> tree.show()  # Display interactively with names
            >>> tree.show("kinematic_tree.png")  # Save to file with names
        """
        if file_name is None:
            file_name = get_sanitized_name(self.cad.name if self.cad.name else "kinematic_graph")

        if graph is None:
            graph = self

        colors = [f"#{random.randint(0, 0xFFFFFF):06x}" for _ in range(len(graph.nodes))]  # noqa: S311
        plt.figure(figsize=(8, 8))
        pos = nx.planar_layout(graph)

        nx.draw(
            graph,
            pos,
            with_labels=True,
            node_color=colors,
            edge_color="white",
            font_color="white",
        )
        plt.savefig(file_name, transparent=True)
        plt.close()
