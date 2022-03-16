# Robot Motion Planning
Robot Motion Planning algorithms allow a robot to find a path from a source to a destination in a region that contains pre-defined obstacles. 

## Open-Loop/Feedforward Planning
Algorithms designed for robots that do not have sensors, but rather have access to a map of the environment or knowledge of the free workspace. Planning is a sequence of pre-computed steps.

## Workspace Decomposition
The workspace is a polygon and each obstacle is a polygon (i.e., a polygonal hole inside the workspace). For complex non-convex environments, the sweeping trapezoidation algorithm is used to decompose the free workspace into the union of convex subsets, such as trapezoids. The trapezoidation of a polygon is the decomposition of the polygon into a collection of trapezoids.

### Sweeping Trapezoidation Algorithm
Consider a workspace in which the boundary is an axis-aligned rectangle and every obstacle vertex has a unique x-coordinate. i.e., no obstacle segment is vertical. Since all x-coordinates are unique, each line segment has a left endpoint and right endpoint, where the x-coordinate of the left endpoint is smaller than that of the right endpoint. To visualize the order in which the vertices are processed a sweeping vertical line is defined moving left to right. When the line hits an environment vertex, it is categorized into one of six types.

<img src="https://user-images.githubusercontent.com/68575242/158691557-1446183e-867b-4d07-97a0-e8e7ff1270ac.png" width="600" height="500">

| Vertex Type | Vertex as Endpoint of Two Segments | Vertex as Convex or Non-Convex | 
| ----------- | --------------------------------- | ------------------------------ |
|     (i)     |            left/left              |            convex              |
|     (ii)    |            left/left              |          non-convex            |
|    (iii)    |           right/right             |            convex              |
|     (iv)    |           right/right             |          non-convex            |
|     (v)     |           left/right              |            convex              |
|     (vi)    |            left/right             |          non-convex            |

A list of obstacle segments intersected by the sweeping line is maintained and changes only when the sweeping line hits a new vertex. This list is then used to extend vertical segments upwards and downwards from the vertex to find intersection points above and blow them (if any). From there the left endpoints of the obstacle segments in the list can be updated and used to add zero, one, or two trapezoids to the set of trapezoids whose union equals the workspace polygon.

## Navigation on Roadmaps
The sweeping trapezoidation algorithm can be easily supplemented to additionally provide a list of neighborhood relationships between trapezoids. As a result of this, an easy-to-navigate roadmap is obtained specified as follows: a collection of center points (one for each trapezoid) and a collection of paths connecting center points (each being composed of 2 segments, connecting a center to midpoint and the same midpoint to a distinct center). Given the decomposition of a workspace into a collection of trapezoids whose edges are defined as follows: there exists an edge between any two trapezoids if and only if the two trapezoids share a vertical segment.

A roadmap is a graph that is described by a set of nodes and a set of edges. The most common way to represent the edge set is via a look-up table, or adjacency table, that is an array whose elements are lists of varying lengths. The lookup table contains the adjacency information as follows: the ith entry in the list is a list of all neighbors of node i. Each node of the graph has an associated polygonal path that connects the two centers through the midpoint of the common vertical segment.

## Search Algorithms Over Graphs
A path in the free workspace that has been decomposed into convex subsets and that is now equipped with a roadmap can be computed by first computing a path in the discrete roadmap.

A path is an ordered sequence of nodes such that from each node there is an edge to the next node in the sequence. The shortest path between two nodes is a path of minimum length between the two nodes. The distance between two nodes is the length of the shortest path connecting them, i.e., the minimum number of edges required to go from one node to the other.

## Breadth-First Search Algorithm
The breadth-first search algorithm is one of the simplest graph search strategies and is optimal in the sense that it computes the shortest paths. The algorithm proceeds in layers. It begins with the start node (Layer 0) and finds all of its neighbors (Layer 1). Then, all unvisited neighbors of Layer 1 (Layer 2) are found, and so on, until we reach a layer that has no unvisited neighbors. Each vertex in layer k+1 is discovered from a parent vertex in layer k. The use of a parent array that contains the node that lies immediately before it, for each node. At the successful completion of BFS, we can use the parent values to define a set of edges for each node for which its parent is defined. These edges define a tree in the graph, forming a connected graph. The extract-path algorithm is then implemented to use the parent values to reconstruct the sequence of nodes on the shortest path from the start node to the goal node. 

If a path exists in the graph from the start to goal point, then the BFS algorithm and the extract-path algorithm will find the shortest such path; if a path does not exist, the algorithm returns a failure notice; in any case, the algorithm completes in a finite number of steps.
