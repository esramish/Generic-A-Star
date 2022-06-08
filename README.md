# Generic-A-Star
A homemade, generalized A* implementation, fit for pathfinding between any kind of nodes that can self-validate, produce their own neighbors, and apply their own heuristic.

### Usage
To apply the algorithm to an arbitrary start-end pair of nodes, you must create a class that implements the `AStarableI` interface, conceptually representing the type of node to which you'll apply the algorithm (e.g., a 2D point). Then simply instantiate a start node and an end node of that type, do any further preparation your node type or conceptual path system requires (e.g., obstacles in a 2D grid), and call `find_path(start, end)`. The call will return an A*-generated list of nodes that represents an optimal path from the start node to the end node (including both of those nodes). See the tests directory for example usage.

# Testing
1. `pip install pytest`
2. From the repo root directory or the tests directory: `pytest` (or `pytest -rP` to view captured stdout)