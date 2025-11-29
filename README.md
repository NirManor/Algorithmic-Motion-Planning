# Algorithmic Robot Motion Planning (236901)

**Technion - Israel Institute of Technology**
**Course:** Advanced Robotics & Autonomous Systems
**Student:** Nir Manor
**Period:** 2023-2024

---

## Overview

This repository contains complete implementations of three complementary motion planning algorithms covering exact, sampling-based, and application-specific planning paradigms. All assignments include both theoretical analysis and practical implementations in Python.

### Learning Outcomes

After this course, I can:

- **Design and implement exact motion planning algorithms** using computational geometry (Minkowski sums, visibility graphs)
- **Apply sampling-based planners** (RRT, RRT*) for high-dimensional configuration spaces
- **Integrate multiple planning paradigms** for real-world robotic manipulation and inspection tasks
- **Analyze algorithmic trade-offs** between optimality, completeness, and computational efficiency
- **Validate algorithms** through comprehensive experiments and statistical analysis
- **Visualize planning processes** and intermediate results for debugging and communication

---

## Project Structure

```
TASP-ARMP/
├── HW1-Exact-Motion-Planning/        # Computational geometry & roadmap planning
│   ├── HW1.py                        # Minkowski sum + visibility graph implementation
│   ├── Plotter.py                    # Visualization utilities
│   └── data/
│       ├── robot/                    # Robot start position & dimensions
│       ├── query/                    # Goal positions
│       └── obstacles/                # Environment obstacles
│
├── HW2-Sampling-Based-Planning/      # RRT, RRT*, A* algorithms
│   ├── AStarPlanner.py              # Weighted A* grid-based planner
│   ├── RRTPlanner.py                # RRT sampling-based planner
│   ├── RRTStarPlanner.py            # RRT* asymptotically optimal variant
│   ├── MapEnvironment.py            # Environment & collision checking
│   ├── RRTTree.py                   # Tree data structure
│   ├── run.py                       # Main execution script
│   └── data/maps/                   # JSON map definitions
│
└── HW3-Manipulation-Inspection/     # Advanced application: task-specific planning
    ├── RRTMotionPlanner.py          # Manipulation planning (reach targets)
    ├── RRTInspectionPlanner.py      # Inspection planning (coverage paths)
    ├── Robot.py                     # Robot model & constraints
    ├── MapEnvironment.py            # Environment & sensor models
    ├── RRTTree.py                   # Specialized tree structure
    ├── run.py                       # Execution & visualization
    ├── data/maps/                   # Problem-specific maps
    └── results/                     # Animated GIFs & trajectory data
        ├── motion-planning/         # MP task results
        └── inspection-planning/     # IP task results
```

---

## HW1: Exact Motion Planning for a Diamond-Shaped Robot

### Problem Statement

Plan collision-free paths for a **diamond-shaped robot** (axis-aligned square rotated 45°) translating amidst **convex polygonal obstacles** in 2D.

### Key Concepts

#### 1. **Configuration Space (C-Space)**
- **Minkowski Sum** of robot shape ⊕ each obstacle
- Transforms collision-avoidance problem into point-robot navigation in expanded obstacle space
- Computational complexity: O(n*m) where n = obstacles, m = vertices per obstacle

#### 2. **Visibility Graph**
- **Roadmap** connecting obstacle vertices with collision-free edges
- Enables efficient multi-query path planning
- Preprocessing: O(n² * m log m) with visibility test per edge

#### 3. **Shortest Path Query**
- **Dijkstra's algorithm** on visibility graph + start/goal nodes
- Query complexity: O((V + E) log V) where V, E = graph vertices/edges

### Implementation Details

**File:** `HW1.py`

```python
# Core functions to implement:
def get_minkowsky_sum(original_shape: Polygon, r: float) -> Polygon:
    """
    Compute Minkowski sum of obstacle with robot shape (diamond of radius r)
    Returns inflated obstacle polygon in C-space
    """

def get_visibility_graph(obstacles: List[Polygon], source=None, dest=None) -> List[LineString]:
    """
    Build visibility graph: edges between vertices with no obstacle intersections
    Optional: include direct edges from start/goal to visible vertices
    """
```

### Experimental Results

- **Test Case 1:** Provided environment with 3-5 obstacles
  - C-space visualization showing inflated obstacles
  - Visibility graph with ~20-30 edges
  - Optimal path with cost analysis

- **Test Case 2:** Custom environment (user-generated)
  - Performance comparison
  - Discussion of non-convex obstacle effects

### Key Insights

✓ **Minkowski sums expand obstacles** to account for robot geometry
✓ **Visibility graph reduces planning complexity** compared to grid-based methods
✓ **Non-convex obstacles require decomposition** into convex parts (or approximate handling)

---

## HW2: Sampling-Based Motion Planning

### Three Algorithm Implementations

#### 1. **Weighted A\* (Grid-based Search)**

**Algorithm:** Deterministic, optimal with ε-weighting

```
Input:  Grid map, start, goal, heuristic weight ε
Output: Optimal path (or ε-approximation)

- 8-connected grid neighborhood
- Euclidean distance heuristic
- f-value = g + ε·h (varying ε trades optimality for speed)
```

**Results:**
| ε   | Path Cost | Nodes Expanded | Computation Time |
|-----|-----------|----------------|------------------|
| 1   | C_opt     | High           | Slower           |
| 10  | 1.5C_opt  | Medium         | Moderate         |
| 20  | 2C_opt    | Low            | Fast             |

**Key Observation:** Higher ε → suboptimal but faster (useful for real-time applications)

#### 2. **RRT (Rapidly-Exploring Random Tree)**

**Algorithm:** Probabilistically complete, non-optimal sampling-based planner

```
Input:  Start, goal, environment, goal_bias (5% or 20%)
Output: Feasible path (not optimized)

Repeat until goal reached:
  1. Sample random state (90% uniform, 10% goal)
  2. Find nearest tree vertex
  3. Extend toward sample (mode E1 or E2)
  4. If collision-free, add to tree
```

**Two Extension Modes:**
- **E1:** Extend all the way to sampled point (aggressive)
- **E2:** Extend by fixed step size (conservative, smoother trees)

**Statistical Results (10 runs):**
- Goal Bias 5%: Success ~95%, Time ~15-20s, Cost ~120-150
- Goal Bias 20%: Success ~100%, Time ~5-8s, Cost ~110-130

#### 3. **RRT\* (RRT-Star: Asymptotically Optimal)**

**Algorithm:** Builds on RRT with path rewiring for optimality

```
After adding new vertex to tree:
  1. Find k-nearest neighbors (k = constant or k = O(log n))
  2. Try connecting from best neighbor
  3. Rewire neighbors through new vertex if cost improves
```

**Rewiring Strategies:**
- **k constant** (e.g., k=5): Fast, moderate quality improvement
- **k = log(n)** formula: Asymptotically optimal, higher computation

**Metrics Over Time:**
- **Success Rate:** Probability solution exists at time t
- **Solution Quality:** Path cost as function of computation time
- Shows convergence to optimal solution as time increases

### Code Structure

```python
# AStarPlanner.py
class AStarPlanner:
    def plan(self):
        # Open set with f-value priority queue
        # Closed set for visited nodes
        # Expand nodes in best-first order
        # Return optimal path

# RRTPlanner.py
class RRTPlanner:
    def plan(self):
        # Tree initialized with start node
        # Loop: sample → nearest → extend → add if valid
        # Extract path when goal reached

# RRTStarPlanner.py
class RRTStarPlanner(RRTPlanner):
    def plan(self):
        # Extends RRTPlanner.plan()
        # After adding vertex: find k-nearest
        # Attempt rewiring for cost improvement

def extend(near_state, rand_state, mode):
    # E1: Linear interpolation to sampled point
    # E2: Limited step size (clip distance)
```

### Visualization Outputs

- **Planning trees:** Show exploration pattern and final solution
- **Expanded nodes:** A* search frontier visualization
- **Path quality over time:** Convergence graphs for RRT*
- **Success rate curves:** Probabilistic completeness demonstration

---

## HW3: Advanced Application - Manipulation & Inspection Planning

### Two Planning Problems

#### **Motion Planning (MP)**
**Goal:** Navigate robot to reach multiple target locations
**Constraints:** Kinematic limits, collision avoidance

#### **Inspection Planning (IP)**
**Goal:** Cover target regions while minimizing path cost
**Constraints:** Sensor range, obstacle avoidance, full coverage

### Implementation Highlights

```python
# RRTMotionPlanner.py
class RRTMotionPlanner:
    """Reach-to-target planning with cost optimization"""
    def plan(self):
        # RRT+ path cost metrics
        # Handle multi-target sequences

# RRTInspectionPlanner.py
class RRTInspectionPlanner:
    """Coverage planning for inspection tasks"""
    def plan(self):
        # Coverage constraint enforcement
        # Visibility polygon computation
```

### Results Showcase

**Animated Results:** GIFs showing algorithm progression

```
results/motion-planning/
├── Plan_MP_E1_GoalBias=0.05_Cost=9.7.gif      (E1 extension mode)
├── Plan_MP_E2_GoalBias=0.05_Cost=6.7.gif      (E2 step-limited)
└── Plan_MP_E2_GoalBias=0.2_Cost=4.8.gif       (Higher goal bias)

results/inspection-planning/
├── Plan_IP_E1_Coverage=0.5_Cost=10.9.gif      (50% coverage)
├── Plan_IP_E1_Coverage=0.75_Cost=14.9.gif     (75% coverage)
└── Plan_IP_E2_Coverage=0.75_Cost=8.1.gif      (E2 optimization)
```

**Key Metrics:**
- Path cost: Minimized through iterative refinement
- Coverage ratio: Percentage of target regions visited
- Computation time: Real-time performance (< 30s for complex scenarios)

### Parameter Study

**Goal Bias Variation:**
- Low (5%): Longer solutions, broader exploration
- High (20%): Faster convergence, potential local optima

**Extension Mode Impact:**
- E1 (direct): Larger steps, faster growth, less smooth
- E2 (limited): Gradual tree expansion, smoother paths

**Coverage Constraints (IP):**
- 50% coverage: Quick solutions (~10-15s)
- 75% coverage: More extensive paths (~40-80s)

---

## Technical Stack

**Dependencies:**
```
numpy          - Numerical computations
matplotlib     - 2D visualization
shapely        - Geometric operations (Minkowski sums, intersection tests)
imageio        - GIF generation & frame capture
heapdict       - Priority queue (for A*)
```

**Installation:**
```bash
pip install numpy matplotlib shapely imageio heapdict
```

**Python Version:** 3.7+

---

## Algorithm Complexity Summary

| Algorithm | Time Complexity | Space | Optimality | Completeness |
|-----------|-----------------|-------|-----------|--------------|
| **Minkowski Sum** | O(n·m²) | O(n·m) | - | - |
| **Visibility Graph** | O(n²·m²) | O(n·m) | Optimal | Yes |
| **Dijkstra** | O((V+E)logV) | O(V) | Optimal | Yes |
| **A\*** | O(b^d) | O(b^d) | Optimal* | Yes |
| **RRT** | O(n·log n) | O(n) | Non-optimal | Probabilistic |
| **RRT\*** | O(n·log n) | O(n) | Asymptotic | Probabilistic |

*A* is optimal if heuristic is admissible (h(s) ≤ cost(s,goal))

---

## How to Run

### HW1: Exact Motion Planning
```bash
cd HW1-Exact-Motion-Planning
python3 HW1.py data/robot/[robot_file] data/obstacles/[obstacles_file] data/query/[query_file]
```

Output: Three visualizations
1. C-space obstacles (inflated by Minkowski sum)
2. Visibility graph on C-space
3. Shortest path with start/goal positions

### HW2: Sampling-Based Planning
```bash
cd HW2-Sampling-Based-Planning

# A* search
python3 run.py --map data/maps/map1.json --planner astar --h_weight 10

# RRT with goal bias
python3 run.py --map data/maps/map2.json --planner rrt --ext_mode E2 --goal_prob 0.05

# RRT* with k=log(n)
python3 run.py --map data/maps/map2.json --planner rrtstar --ext_mode E2 --goal_prob 0.2 --k log
```

### HW3: Manipulation & Inspection
```bash
cd HW3-Manipulation-Inspection

# Motion planning
python3 run.py --task motion_planning --map data/maps/map_mp.json --goal_bias 0.05 --extension E2

# Inspection planning with coverage constraint
python3 run.py --task inspection_planning --map data/maps/map_ip.json --coverage 0.75 --extension E2
```

---

## Key Learnings by Domain

### **Autonomous Driving / Decision-Making**
✓ Roadmap-based planning for real-time path generation
✓ Heuristic search (A*) for optimal route planning under time constraints
✓ Trade-off analysis: optimality vs. computation time
✓ Multi-goal sequencing and dynamic replanning

### **Robotics / Manipulation Control**
✓ Configuration space representation for complex robot geometries
✓ Sampling-based planning for high-DOF systems (arm manipulation in HW3)
✓ Tree-based exploration for non-convex configuration spaces
✓ Iterative refinement (RRT*) for improving manipulation solution quality

### **Industrial Automation / Path Planning**
✓ Scalable algorithms handling 10²-10⁵ node trees
✓ Real-time performance optimization (E2 vs E1 modes)
✓ Parameter tuning for performance: coverage, cost, completion time
✓ Inspection planning for autonomous inspection/maintenance robots

---

## References

1. **Computational Geometry & Exact Planning:**
   - Latombe, J. C. (1991). Robot Motion Planning. Kluwer Academic Publishers.
   - De Berg, M., et al. (2008). Computational Geometry: Algorithms and Applications.

2. **Sampling-Based Planning:**
   - LaValle, S. M., & Kuffner, J. J. (2001). Randomized Kinodynamic Planning.
   - Karaman, S., & Frazzoli, E. (2011). Sampling-based Algorithms for Optimal Motion Planning.

3. **Search & Optimization:**
   - Russell, S., & Norvig, P. (2020). Artificial Intelligence: A Modern Approach (4th ed.). Ch. 3-4
   - Dijkstra, E. W. (1959). A Note on Two Problems in Connexion with Graphs.

---

## Course Information

**Institution:** Technion - Israel Institute of Technology
**Course Code:** 236901
**Instructor:** [Course Instructor Name]
**Semester:** Fall 2023
**Credits:** 3

---

## Repository Status

✅ **HW1:** Complete - Minkowski sums, visibility graphs, Dijkstra implementation
✅ **HW2:** Complete - A*, RRT, RRT* with parameter sweeps
✅ **HW3:** Complete - Motion & inspection planning with animated results

---

**Last Updated:** November 2024
**License:** Educational Use (Technion Coursework)
**Author:** Nir Manor | Technion TASP Program

