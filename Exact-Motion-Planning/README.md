# Exact Motion Planning: Computational Geometry & Roadmap Planning

Implementation of exact motion planning algorithms using computational geometry for a diamond-shaped robot navigating through 2D polygonal obstacle environments.

## Problem Description

Navigate a **diamond-shaped robot** (axis-aligned square rotated 45°) translating amidst **convex polygonal obstacles** in 2D, computing collision-free optimal paths.

**Key Requirements:**
- Compute configuration space (C-space) through obstacle inflation
- Build visibility graph as a roadmap
- Find optimal collision-free paths using Dijkstra's algorithm
- Support multiple queries on the same environment

**Constraints:**
- Robot is a diamond shape with specified dimensions
- Obstacles are convex polygons
- Motion is purely translational (no rotation)
- Must find optimal (shortest) paths

---

## Algorithms Implemented

### 1. **Minkowski Sum (C-Space Computation)**

**Purpose:** Transform collision-avoidance problem into point-robot navigation

The Minkowski sum of each obstacle with the robot shape creates an expanded obstacle in configuration space. This transforms the problem of a robot of size navigating in the original space into a point robot navigating in the expanded space.

**Mathematical Foundation:**
- Minkowski sum: O ⊕ R = {o + r | o ∈ O, r ∈ R}
- For convex obstacles and robot, result is convex
- Complexity: O(n·m²) where n = obstacles, m = vertices per obstacle

**Implementation Details:**
- Diamond robot represented as 4 vertices at radius r
- Obstacle vertices inflated by robot shape
- Result: Enlarged obstacles that represent forbidden regions

**Key Insight:**
Expanding obstacles by robot shape converts the motion planning problem from a robot of size navigating in original space to a point navigating in expanded space.

---

### 2. **Visibility Graph (Roadmap Construction)**

**Purpose:** Build compact roadmap connecting start, goal, and obstacle vertices

The visibility graph includes the start and goal vertices along with all obstacle vertices. For each pair of vertices, the algorithm tests if a straight line connecting them intersects any obstacle. If clear, an edge is added with cost equal to the Euclidean distance.

**Preprocessing Complexity:** O(n²·m²)
- n² vertex pairs
- m² edge-vertex intersection tests per pair

**Key Properties:**
- Optimal paths lie along visibility graph edges
- Includes edges to/from start and goal vertices
- Handles multiple obstacles efficiently

**Implementation Details:**
- Edge validity checking using geometric intersection tests
- Start/goal vertices visible from appropriate obstacle vertices
- Undirected graph with Euclidean distances as edge weights

---

### 3. **Dijkstra's Algorithm (Shortest Path Query)**

**Purpose:** Find minimum-cost path from start to goal on visibility graph

Dijkstra's algorithm initializes start distance to 0 with all others set to infinity, then repeatedly extracts the minimum-distance vertex and relaxes its neighbors. Path reconstruction follows parent pointers from goal back to start, guaranteeing optimality on the visibility graph.

**Complexity:** O((V+E)log V)
- V = visibility graph vertices (obstacle vertices + 2)
- E = visibility graph edges
- log V factor from priority queue operations

**Query Properties:**
- Single-source, multiple-destination search
- Optimality guaranteed on visibility graphs
- Efficient for multiple queries on same environment


**File:** `Plotter.py`

Visualization utilities for displaying:
- Original obstacles and robot
- C-space expanded obstacles
- Visibility graph structure
- Final collision-free path

Methods:
- `add_obstacles()` - Plot original environment
- `add_c_space_obstacles()` - Show inflated obstacles
- `add_visibility_graph()` - Display roadmap
- `add_shortest_path()` - Show optimal solution

---

## Results

### Experiment 1: Test Environment (Provided)

**Setup:**
- 3-5 convex polygonal obstacles
- Diamond robot with specified dimensions
- Random start and goal positions

**Results:**
- ✓ C-space computation successful
- ✓ Visibility graph contains ~20-30 edges
- ✓ Dijkstra finds optimal path
- ✓ Path visualization shows collision-free navigation

### Experiment 2: Custom Environment

**Setup:**
- User-defined obstacle configuration
- Varying obstacle densities and geometries

**Key Observations:**
- Algorithm handles convex obstacles reliably
- Performance scales with number of vertices
- Non-convex obstacles require decomposition (not implemented)

---

## Complexity Analysis

| Operation | Complexity | Notes |
|-----------|-----------|-------|
| Minkowski Sum | O(n·m²) | n obstacles, m vertices each |
| Visibility Test | O(m²) per edge | Each edge vs each obstacle |
| Graph Construction | O(n²·m²) | All vertex pairs × intersection tests |
| Dijkstra Query | O((V+E)log V) | V = O(n·m), E = O(n²) |
| **Total Preprocessing** | **O(n²·m²)** | One-time cost |
| **Per-Query** | **O(n²·m² log(n·m))** | If including all edges, amortized |

---

## Algorithm Trade-offs

### Advantages
✓ **Optimality:** Guaranteed to find shortest path
✓ **Efficiency:** Preprocessing enables fast multiple queries
✓ **Completeness:** Finds path if one exists among convex obstacles
✓ **Deterministic:** No randomness, reproducible results

### Disadvantages
✗ **Convexity requirement:** Non-convex obstacles need decomposition
✗ **Preprocessing cost:** Expensive for complex environments
✗ **2D limitation:** Hard to extend to 3D (quadratic visibility)
✗ **High dimensions:** Not applicable to high-DOF robots

---

## Comparison: Visibility Graphs vs Sampling-Based Planning

| Aspect | Visibility Graph | RRT/RRT* |
|--------|------------------|----------|
| **Optimality** | Guaranteed | Asymptotic (RRT*) |
| **Obstacle Type** | Convex only | Arbitrary shapes |
| **Preprocessing** | Required | None |
| **Multiple Queries** | Fast | Each query is independent |
| **Scalability** | Limited by visibility | Scales to high-dimensions |
| **Complexity** | O(n²·m²) | O(n log n) |
| **Implementation Difficulty** | Medium | Low |

---

## Key Learnings

### Computational Geometry Concepts
- Minkowski operations for robot-obstacle interaction
- Geometric collision detection
- Visibility computation and line-polygon intersection
- Optimal roadmap construction

### Algorithm Design Patterns
- Preprocessing-based speedup for multiple queries
- Graph-based path planning formulation
- Optimal path extraction via Dijkstra
- Complexity-optimality trade-offs

### Practical Implementation
- Polygon representation (vertices, edges)
- Floating-point geometric computations
- Efficiency vs. correctness in geometric algorithms
- Visualization for algorithm understanding

---

## Files

- `HW1.py` - Main implementation (Minkowski sum, visibility graph, Dijkstra)
- `Plotter.py` - Visualization utilities
- `assignment.pdf` - Full problem statement with formal specifications
- `report.pdf` - Solution report with results and analysis
- `data/robot/` - Robot configuration files
- `data/obstacles/` - Obstacle definitions
- `data/query/` - Query (start/goal) positions

---

## Usage

```bash
cd Exact-Motion-Planning
python HW1.py data/robot/[robot_file] data/obstacles/[obstacles_file] data/query/[query_file]
```

Expected Output:
- Visualization 1: Original obstacles and robot
- Visualization 2: C-space inflated obstacles
- Visualization 3: Visibility graph on C-space
- Visualization 4: Optimal path from start to goal

---

**Status:** ✅ Complete with comprehensive documentation
**Last Updated:** 2025
**Topics:** Computational Geometry, Roadmap Planning, Optimal Search
