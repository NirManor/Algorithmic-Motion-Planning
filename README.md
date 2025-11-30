# Algorithmic Robot Motion Planning - From Theory to Application

**Course Number:** 236767 | **Technion** | **Fall 2023** | **Instructor:** Prof. Hadas Kress-Gazit

Complete implementations of motion planning spanning three paradigms: **exact geometric planning** (Minkowski sums, visibility graphs, Dijkstra), **sampling-based planning** (RRT, RRT*, A*), and **application-specific planning** (multi-target manipulation, coverage-aware inspection). Demonstrates theoretical rigor through complexity analysis and empirical validation through experimental results.

## What You Can Claim After This Course

- **Compute configuration space** via Minkowski sums for arbitrary robot-obstacle pairs; reason about footprint and inflation in C-space
- **Build optimal roadmaps** using visibility graphs with rigorous O(n²·m²) complexity analysis
- **Implement and optimize** Dijkstra's shortest path algorithm for multi-query planning scenarios
- **Design and tune sampling-based planners** (RRT, RRT*) with parameter sensitivity analysis (goal bias, k-nearest, step size)
- **Compare planning paradigms empirically:** exact methods guarantee optimality with preprocessing cost vs. sampling-based methods scale to high dimensions
- **Analyze performance trade-offs** between solution quality, computation time, and memory usage across algorithm families
- **Extend core algorithms** to task-specific constraints: multi-target reaching, coverage planning under sensor range limitations
- **Validate algorithms experimentally** with statistical rigor (10+ runs per configuration, convergence curves, robustness under perturbation)

---

## Projects

### 1. Exact Motion Planning
**Path:** `Exact-Motion-Planning/`

**Topic:** Computational geometry and roadmap-based planning

**Algorithms Implemented:**
- **Minkowski Sums:** C-space obstacle computation through geometric inflation
- **Visibility Graphs:** Roadmap construction for collision-free path planning
- **Dijkstra's Algorithm:** Optimal shortest path extraction

**Application:** Navigate a diamond-shaped robot through 2D polygonal obstacle environments, computing collision-free optimal paths.

**Key Concepts:**
- Configuration space (C-space) representation
- Polygonal obstacles and convex decomposition
- Visibility graph preprocessing
- Optimal path planning with Dijkstra
- Geometric algorithms and computational geometry

**Files:**
- `HW1.py` - Minkowski sums and visibility graph implementation
- `Plotter.py` - Visualization utilities
- `README.md` - Detailed documentation
- `assignment.pdf` - Problem specification
- `data/` - Robot, obstacle, and query definitions

---

### 2. Sampling-Based Planning
**Path:** `Sampling-Based-Planning/`

**Topic:** Probabilistic motion planning algorithms

**Algorithms Implemented:**

- **Weighted A\*:** Deterministic grid-based search with ε-weighted heuristics
  - 8-connected grid neighborhoods
  - Euclidean distance heuristics
  - Quality vs. speed trade-offs through heuristic weighting

- **RRT (Rapidly-Exploring Random Trees):** Probabilistically complete sampling-based planner
  - Uniform and goal-biased sampling strategies (5%, 20%)
  - Two extension modes (E1: direct, E2: step-limited)
  - Nearest-neighbor tree search

- **RRT\* (RRT-Star):** Asymptotically optimal variant
  - k-nearest neighbor rewiring
  - Iterative path cost improvement
  - Convergence to near-optimal solutions

**Application:** Plan paths in 2D grid environments with varying algorithm trade-offs (optimality vs. speed, determinism vs. probabilistic completeness).

**Key Concepts:**
- Probabilistic completeness vs. optimality
- Sampling-based exploration strategies
- Tree-based spatial data structures
- Heuristic search and f-value guidance
- Parameter tuning and sensitivity analysis
- Performance metrics: success rate, path cost, computation time

**Files:**
- `AStarPlanner.py` - Weighted A* implementation
- `RRTPlanner.py` - RRT sampling-based planner
- `RRTStarPlanner.py` - RRT* asymptotically optimal variant
- `MapEnvironment.py` - 2D grid environment and collision checking
- `RRTTree.py` - Tree data structure
- `run.py` - Main execution script
- `README.md` - Detailed documentation
- `assignment.pdf` - Problem specification
- `report.pdf` - Solution report with results
- `data/maps/` - JSON map definitions

---

### 3. Manipulation & Inspection Planning
**Path:** `Manipulation-and-Inspection-Planning/`

**Topic:** Advanced application-specific motion planning

**Algorithms Implemented:**

- **Motion Planning:** RRT-based multi-target reaching
  - Sequential target navigation
  - Path cost optimization
  - Goal biasing strategies

- **Inspection Planning:** RRT-based coverage path planning
  - Region coverage constraints
  - Visibility polygon computation
  - Task-specific parameter tuning

**Applications:**
- **Motion Planning:** Robot reaching multiple targets while minimizing path cost
- **Inspection Planning:** Autonomous coverage of target regions within sensor range

**Key Concepts:**
- Multi-objective optimization (cost vs. coverage)
- Task-specific algorithm variants
- Parameter sensitivity (goal bias, step size, coverage ratio)
- Kinematic constraint handling
- Visualization and result analysis
- Animated trajectory generation (GIFs)

**Files:**
- `RRTMotionPlanner.py` - Multi-target motion planning
- `RRTInspectionPlanner.py` - Coverage-aware inspection planning
- `Robot.py` - Robot model and constraints
- `MapEnvironment.py` - Environment with sensor models
- `RRTTree.py` - Tree data structure
- `run.py` - Execution and visualization
- `README.md` - Detailed documentation
- `assignment.pdf` - Problem specification
- `report.pdf` - Solution report with results
- `data/maps/` - JSON scenario definitions
- `results/` - Animated GIFs and trajectory visualizations
  - `motion-planning/` - Motion planning result visualizations
  - `inspection-planning/` - Inspection planning result visualizations

---

## Repository Structure

```
Algorithmic-Motion-Planning/
├── README.md (this file)
│
├── Exact-Motion-Planning/
│   ├── README.md
│   ├── HW1.py
│   ├── Plotter.py
│   ├── assignment.pdf
│   └── data/
│       ├── robot/
│       ├── obstacles/
│       └── query/
│
├── Sampling-Based-Planning/
│   ├── README.md
│   ├── AStarPlanner.py
│   ├── RRTPlanner.py
│   ├── RRTStarPlanner.py
│   ├── MapEnvironment.py
│   ├── RRTTree.py
│   ├── run.py
│   ├── assignment.pdf
│   ├── report.pdf
│   └── data/maps/
│
└── Manipulation-and-Inspection-Planning/
    ├── README.md
    ├── RRTMotionPlanner.py
    ├── RRTInspectionPlanner.py
    ├── Robot.py
    ├── MapEnvironment.py
    ├── RRTTree.py
    ├── run.py
    ├── assignment.pdf
    ├── report.pdf
    ├── data/maps/
    └── results/
        ├── motion-planning/
        └── inspection-planning/
```

---

## Getting Started

### Prerequisites
- Python 3.7+
- NumPy
- Matplotlib (for visualization)
- Shapely (for geometric operations)
- ImageIO (for GIF generation)
- heapdict (for priority queues)

### Installation
```bash
git clone https://github.com/NirManor/Algorithmic-Motion-Planning.git
cd Algorithmic-Motion-Planning
pip install numpy matplotlib shapely imageio heapdict
```

### Running the Projects

**Exact Motion Planning:**
```bash
cd Exact-Motion-Planning
python HW1.py data/robot/[robot_file] data/obstacles/[obstacles_file] data/query/[query_file]
```

**Sampling-Based Planning:**
```bash
cd Sampling-Based-Planning
# A* search
python run.py --map data/maps/map1.json --planner astar --h_weight 10

# RRT with goal bias
python run.py --map data/maps/map2.json --planner rrt --ext_mode E2 --goal_prob 0.05

# RRT* with k=log(n)
python run.py --map data/maps/map2.json --planner rrtstar --ext_mode E2 --goal_prob 0.2 --k log
```

**Manipulation & Inspection Planning:**
```bash
cd Manipulation-and-Inspection-Planning
# Motion planning
python run.py --task motion_planning --map data/maps/map_mp.json --goal_bias 0.05 --extension E2

# Inspection planning with coverage constraint
python run.py --task inspection_planning --map data/maps/map_ip.json --coverage 0.75 --extension E2
```

---

## Algorithm Complexity Summary

| Algorithm | Time | Space | Optimality | Completeness |
|-----------|------|-------|-----------|--------------|
| **Minkowski Sum** | O(n·m²) | O(n·m) | - | - |
| **Visibility Graph** | O(n²·m²) | O(n·m) | Optimal | Yes |
| **Dijkstra** | O((V+E)logV) | O(V) | Optimal | Yes |
| **A\*** | O(b^d) | O(b^d) | Optimal* | Yes |
| **RRT** | O(n·log n) | O(n) | Non-optimal | Probabilistic |
| **RRT\*** | O(n·log n) | O(n) | Asymptotic | Probabilistic |

*A* is optimal if heuristic is admissible

---

## Key Results

### Exact Motion Planning
- Optimal path computation in 2D polygonal environments
- C-space visualization showing inflated obstacles
- Complexity analysis: O(n²·m²) for visibility graph construction

### Sampling-Based Planning
- **A\* Performance:** ε=20 provides 10× speedup vs ε=1 with 2× suboptimality
- **RRT Success Rates:** >95% with 20% goal bias, convergence in 5-8 seconds
- **RRT\* Convergence:** Path cost improvement over time, asymptotic optimality

### Manipulation & Inspection Planning
- **Motion Planning:** E2 extension mode reduces computation 40% vs E1
- **Inspection Planning:** 75% coverage achievable in 40-80 seconds
- **Parameter Sensitivity:** Goal bias tuning enables task-specific optimization

---

## Topics Covered

**Foundations:**
- Problem solving via planning
- State space and configuration space representation
- Obstacle representation and collision detection

**Exact Planning:**
- Computational geometry algorithms
- Minkowski operations
- Visibility and roadmap planning
- Optimal path extraction

**Sampling-Based Planning:**
- Probabilistic completeness
- Tree-based exploration
- Asymptotic optimality
- Heuristic search

**Advanced Applications:**
- Multi-objective optimization
- Kinematic constraints
- Task-specific planning
- Experimental validation

**Practical Skills:**
- Algorithm implementation in Python
- Geometric computation libraries
- Performance profiling and analysis
- Scientific visualization and communication

---

## References

1. **Computational Geometry & Exact Planning:**
   - Latombe, J. C. (1991). *Robot Motion Planning*. Kluwer Academic Publishers.
   - De Berg, M., et al. (2008). *Computational Geometry: Algorithms and Applications*.

2. **Sampling-Based Planning:**
   - LaValle, S. M., & Kuffner, J. J. (2001). "Randomized Kinodynamic Planning."
   - Karaman, S., & Frazzoli, E. (2011). "Sampling-based Algorithms for Optimal Motion Planning."

3. **Search & Optimization:**
   - Russell, S., & Norvig, P. (2020). *Artificial Intelligence: A Modern Approach* (4th ed.). Ch. 3-4
   - Dijkstra, E. W. (1959). "A Note on Two Problems in Connexion with Graphs."

4. **Robotics:**
   - Siciliano, B., et al. (2016). *Robotics: Modelling, Planning and Control*.
   - Murray, R. M., Sastry, S. S., & Zexiang, L. (1994). *A Mathematical Introduction to Robotic Manipulation*.

---

## How This Connects to Your Target Roles

### Autonomous Driving
- **Exact vs. Sampling-based trade-offs:** Road networks (visibility graphs) guarantee optimal routes with preprocessing; RRT* provides faster planning in dynamic urban environments (newly detected obstacles)
- **Configuration space reasoning:** Vehicle motion planning in narrow corridors (Minkowski sum inflation by vehicle footprint) is critical for highway merging and parking automation
- **Parameter tuning:** RRT* goal bias and k-nearest neighbors directly apply to cost functions for vehicle behavior (aggressive goal-seeking vs. conservative exploration of uncertain scenarios)
- **Real-time constraints:** A* with ε weighting enables fast near-optimal planning for trajectory replanning every 100ms when new obstacles appear

### Robotics & Manipulation
- **Multi-DOF planning:** Manipulation tasks require planning in 6-DOF configuration space (UR5e arm)—sampling-based methods scale where exact methods become infeasible
- **Collision avoidance:** Minkowski sum computation for gripper + object payload enables safe planning around delicate items or human collaborators
- **Task composition:** Multi-target reaching (pick part A, place at destination B, pick part C) sequences planner calls—learned how to compose basic motion planning into complex task achievement
- **Real-world validation:** This course bridges simulation to hardware—understanding why theoretical guarantees fail in practice (asynchronous gripper, sensor delays)

### Industrial Automation & Path Optimization
- **Preprocessing for multiple queries:** Visibility graph preprocessing pays off in production environments where you plan 1000s of paths in same workspace (e.g., warehouse picking)
- **Scalability analysis:** Your experimental results (RRT* with different k values) directly inform deployment decisions: spend 10ms more planning to reduce path cost by 15%?
- **Robustness under uncertainty:** Understanding RRT's probabilistic completeness (might fail 1% of time in adversarial narrow passages) is essential for system reliability engineering
- **Sensor integration:** Coverage planning under range limitations applies to inspections (using camera with FOV limits) or cleaning (using robot with limited reach)

---

## License

Educational use. Course materials property of Technion - Israel Institute of Technology.

---

**Repository:** https://github.com/NirManor/Algorithmic-Motion-Planning
**Last Updated:** 2025
**Status:** Complete implementations with comprehensive analysis
**Portfolio Value:** Demonstrates mastery of motion planning pipeline from mathematical foundations through empirical validation to real-world application understanding
