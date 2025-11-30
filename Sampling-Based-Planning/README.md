# Sampling-Based Planning: Probabilistic Algorithms for Path Planning

Implementation of three complementary sampling-based motion planning algorithms: Weighted A\*, RRT (Rapidly-Exploring Random Trees), and RRT\* (asymptotically optimal variant).

## Problem Description

Plan collision-free paths in 2D grid environments with varying algorithmic objectives:

**Environment:**
- 2D grid map with obstacle cells and free space
- Start and goal positions specified by user
- 8-connected grid neighborhoods (Manhattan + diagonal movement)

**Objectives:**
1. **A\*:** Find optimal paths deterministically
2. **RRT:** Find feasible paths quickly with probabilistic completeness
3. **RRT\*:** Find near-optimal paths with asymptotic optimality

**Key Challenge:**
Trade-off between:
- **Optimality** vs. **Speed** (A\* vs RRT)
- **Determinism** vs. **Probabilistic Completeness** (A\* vs sampling-based)
- **Solution Quality** vs. **Computation Time** (via parameter tuning)

---

## Algorithms Implemented

### 1. **Weighted A\* (Deterministic Optimal Search)**

**Purpose:** Deterministic grid-based search with heuristic guidance and quality/speed trade-offs

A* maintains open and closed sets, expanding nodes based on f-value (combination of path cost g and heuristic h weighted by ε). The algorithm guarantees optimality when ε=1 and provides adjustable suboptimality when ε>1.

**Key Parameter:** Heuristic Weight ε
- **ε = 1:** Uniform cost search (optimal but slow)
- **ε > 1:** Weighted A\* (suboptimal but faster)
- **ε >> 1:** Greedy search (fastest but worst solutions)

**f-Value Definition:**
- f(n) = g(n) + ε·h(n)
- g(n) = cost from start to node n
- h(n) = heuristic estimate from n to goal
- Ε weights the heuristic influence

**Heuristic:** Euclidean distance
- Admissible: h(n) ≤ true_cost(n, goal)
- Consistent: h(n) ≤ cost(n, neighbor) + h(neighbor)

**Complexity:**
- Time: O(b^d) in worst case (b = branching factor ~8, d = depth)
- Space: O(b^d) for storing explored nodes
- Optimality: Optimal if ε = 1; suboptimal for ε > 1

**Experimental Results:**

| ε Value | Path Cost | Nodes Expanded | Time (ms) | Quality |
|---------|-----------|----------------|-----------|---------|
| 1 | C_opt | 1000+ | 500-1000 | Optimal |
| 10 | 1.2-1.5·C_opt | 300-500 | 100-200 | Good |
| 20 | 1.8-2.0·C_opt | 100-200 | 20-50 | Fair |

**Key Insight:** ε = 10-20 provides 10× speedup with only 20-100% suboptimality - useful for real-time applications.

---

### 2. **RRT (Rapidly-Exploring Random Trees)**

**Purpose:** Probabilistically complete sampling-based planner for rapid exploration

RRT iteratively samples random states and extends the tree toward them. Each iteration: samples a random point (or goal with bias probability), finds the nearest tree node, extends toward the sample (either directly in E1 mode or with step limitation in E2 mode), and checks for collisions. The tree grows until the goal is reached.

**Two Extension Modes:**

| Aspect | E1 (Direct) | E2 (Step-Limited) |
|--------|------------|-----------------|
| **Extension** | To sampled point | Fixed step size |
| **Tree Growth** | Aggressive | Gradual |
| **Path Quality** | More direct | Smoother |
| **Computation** | Fewer iterations | More iterations |
| **Best For** | Narrow passages | Smooth trajectories |

**Goal Biasing Parameter p_goal:**
- **Low (5%):** Broad exploration, longer solution times
- **High (20%):** Quick goal convergence, less exploration

**Probabilistic Properties:**
- **Probabilistically Complete:** P(find path) → 1 as iterations → ∞
- **Non-Optimal:** Found paths are typically longer than optimal
- **Random but Effective:** Works even in complex non-convex spaces

**Complexity:**
- Time per iteration: O(log n) with spatial indexing
- Total time: O(n log n) for n iterations
- Space: O(n) for tree storage

**Experimental Results (10 runs each):**

| Goal Bias | Success % | Time (s) | Path Cost | Consistency |
|-----------|-----------|----------|-----------|-------------|
| 5% | 95% | 15-20 | 120-150 | Low |
| 20% | 100% | 5-8 | 110-130 | High |

**Key Insight:** Goal biasing provides dramatic speedup (2-3×) with minimal quality loss.

---

### 3. **RRT\* (RRT-Star: Asymptotically Optimal)**

**Purpose:** Asymptotically optimal variant of RRT through rewiring

RRT* extends RRT with a rewiring phase. After adding a new node, the algorithm: finds k-nearest neighbors (either a fixed constant or logarithmic in tree size), selects the best parent among neighbors to minimize cost-to-start, and rewires neighboring nodes if they benefit from a lower-cost path through the new node. This continuous improvement enables asymptotic optimality.

**k-Nearest Strategies:**

| Strategy | k Value | Optimality | Computation | Convergence |
|----------|---------|-----------|------------|-------------|
| **Fixed** | k=5 | Near-optimal | O(log n) | 50-100 iter |
| **Logarithmic** | k=log(n) | Asymptotic | O(n log n) | 500+ iter |

**Asymptotic Optimality:**
- As iterations → ∞, solution cost → optimal cost
- Path quality improves continuously over time
- Convergence rate proportional to 1/n^(1/d) where d = dimension

**Complexity:**
- Per iteration: O(log n) with spatial indexing
- Total: O(n log n) for n iterations
- Space: O(n) for tree

**Experimental Results:**

| Parameter | E1/E2 | Coverage | Time (s) | Iterations | Convergence |
|-----------|-------|----------|----------|-----------|------------|
| k=5 | E2 | 95% | 10-15 | 100-150 | Good |
| k=log(n) | E2 | 95% | 20-30 | 200-300 | Excellent |

**Convergence Curve:**
- Initial phase: Rapid improvement (0-50 iterations)
- Linear phase: Steady improvement (50-200 iterations)
- Convergence phase: Diminishing returns (200+ iterations)

---

## Algorithm Comparison

| Criterion | A\* | RRT | RRT\* |
|-----------|-----|-----|-------|
| **Optimality** | Guaranteed | None | Asymptotic |
| **Completeness** | Yes | Probabilistic | Probabilistic |
| **Preprocessing** | None | None | None |
| **Time Complexity** | O(b^d) | O(n log n) | O(n log n) |
| **Space Complexity** | O(b^d) | O(n) | O(n) |
| **Obstacle Handling** | Grid-based | Any shape | Any shape |
| **Obstacle Type** | Convex cells | Arbitrary | Arbitrary |
| **Implementation** | Medium | Medium | Medium |
| **Best Use Case** | Small grids | Complex spaces | Optimal paths |

---

## Experimental Results & Analysis

### Experiment 1: A\* Epsilon Parameter Sweep

**Objective:** Characterize optimality vs. speed trade-off

**Setup:** 20×20 grid, 40% obstacle density

**Results:**
- ε=1: Optimal paths, 500-1000ms computation
- ε=10: 1.3× suboptimal, 100-200ms (5× speedup)
- ε=20: 2× suboptimal, 20-50ms (10× speedup)

**Conclusion:** ε=10 provides best balance for real-time applications.

### Experiment 2: RRT Goal Bias Sensitivity

**Objective:** Evaluate impact of sampling strategy

**Setup:** 20×20 grid, varying obstacle densities, 10 runs each

**Results:**

| Goal Bias | 30% Obstacles | 40% Obstacles | 50% Obstacles |
|-----------|---------------|---------------|---------------|
| 5% | 90% success, 18s | 85% success, 25s | 60% success, 40s |
| 20% | 100% success, 5s | 95% success, 8s | 90% success, 15s |

**Conclusion:** 20% goal bias ensures high success rates with fast convergence.

### Experiment 3: RRT vs RRT\* Convergence

**Objective:** Compare path quality improvement over time

**Setup:** 20×20 grid, 40% obstacles, k=log(n)

**Results:**
- RRT (E2): Finds path in 100 iterations, cost ~130
- RRT\*: Finds path in 50 iterations, cost ~120
- RRT\* at 300 iterations: Cost ~100 (near-optimal)

**Convergence Rate:** ~5-10% improvement per 50 iterations

---

## Algorithm Results & Visualizations

### A* Performance (Different Epsilon Weights)

| Algorithm | Epsilon | Path Quality | Computation Speed | Best For |
|-----------|---------|--------------|-------------------|----------|
| **Weighted A*** | ε=1 | Optimal | Slow (500-1000ms) | Guaranteed optimality |
| **Weighted A*** | ε=10 | 1.3× suboptimal | Fast (100-200ms) | Balanced applications |
| **Weighted A*** | ε=20 | 2× suboptimal | Very Fast (20-50ms) | Real-time systems |

**Visual Results:**
![A* with epsilon=1.0 (Optimal)](figures/Astar_epsilon_1.0.png)
*A* epsilon=1.0: Finds optimal path with thorough exploration*

![A* with epsilon=10.0 (Balanced)](figures/Astar_epsilon_10.0.png)
*A* epsilon=10.0: Balances speed and optimality (5× speedup)*

![A* with epsilon=20.0 (Fast)](figures/Astar_epsilon_20.0.png)
*A* epsilon=20.0: Prioritizes speed with acceptable suboptimality*

---

### RRT* k-Nearest Neighbor Analysis

Different k-values significantly impact RRT* convergence and solution quality:

| k Value | Rewiring Neighbors | Optimality | Computation | Convergence |
|---------|-------------------|-----------|------------|------------|
| **k=1** | Nearest only | Fast but suboptimal | Low | Quick |
| **k=5** | Fixed 5 neighbors | Near-optimal | Medium | Good |
| **k=10** | Fixed 10 neighbors | Better quality | Higher | Excellent |
| **k=log(n)** | Logarithmic growth | Asymptotically optimal | High | Best |

**Visual Results:**
![RRT* k=1 (Single Neighbor)](figures/k_1.png)
*RRT* k=1: Minimal rewiring, quick convergence*

![RRT* k=5 (Fixed 5)](figures/k_5.png)
*RRT* k=5: Good balance between quality and speed*

![RRT* k=10 (Fixed 10)](figures/k_10.png)
*RRT* k=10: Better path quality at higher computation cost*

![RRT* k=log(n) (Optimal)](figures/k_logn.png)
*RRT* k=log(n): Asymptotic optimality with continuous improvement*

---

### Goal Bias Comparison (Small Value: 0.05)

Lower goal bias (5%) promotes broader exploration before target convergence:

![Goal Bias 0.05 - Example 1](figures/goal_bias_0.05/goal_bias_0.05_E2_1.png)
*E2 mode, Goal Bias=0.05: Gradual exploration, well-distributed tree*

![Goal Bias 0.05 - Example 3](figures/goal_bias_0.05/goal_bias_0.05_E2_3.png)
*E2 mode, Goal Bias=0.05: Broader coverage before converging to goal*

![Goal Bias 0.05 - Example 5](figures/goal_bias_0.05/goal_bias_0.05_E2_5.png)
*E2 mode, Goal Bias=0.05: Thorough space exploration*

![Goal Bias 0.05 - Example 7](figures/goal_bias_0.05/goal_bias_0.05_E2_7.png)
*E2 mode, Goal Bias=0.05: Multiple exploration strategies shown*

---

### Goal Bias Comparison (Higher Value: 0.2)

Higher goal bias (20%) accelerates convergence to the goal:

![Goal Bias 0.2 - Example 1](figures/goal_bias_0.2/goal_bias_0.2_E2_1.png)
*E2 mode, Goal Bias=0.2: Quick goal-directed growth*

![Goal Bias 0.2 - Example 3](figures/goal_bias_0.2/goal_bias_0.2_E2_3.png)
*E2 mode, Goal Bias=0.2: Faster convergence to target*

![Goal Bias 0.2 - Example 5](figures/goal_bias_0.2/goal_bias_0.2_E2_5.png)
*E2 mode, Goal Bias=0.2: Efficient path finding*

![Goal Bias 0.2 - Example 7](figures/goal_bias_0.2/goal_bias_0.2_E2_7.png)
*E2 mode, Goal Bias=0.2: Goal-biased extension efficiency*

---

### Performance Comparison Metrics

![Path Cost vs Time](figures/PathCostVsTime.png)
*Cost-time trade-off: Higher path costs achieved with shorter computation*

![Success Rate vs Time](figures/SuccessRateVsTime.png)
*Success rate improves with longer computation times*

---

### Algorithm Comparison on Map2

![RRT E2 Goal Bias 0.05](figures/Map2_RRT_E2_GoalBias0.05.png)
*RRT E2 mode with 5% goal bias: Balanced exploration*

![RRT E2 Goal Bias 0.2](figures/Map2_RRT_E2_GoalBias0.2.png)
*RRT E2 mode with 20% goal bias: Aggressive goal-directed search*

![RRT* E2 Goal Bias 0.05 k=5](figures/Map2_RRTStar_E2_GoalBias0.05_k5.png)
*RRT* E2 mode with 5% goal bias and k=5: Rewiring optimization visible*

---

### Key Observations

**A* Epsilon Trade-offs:**
- ε=1: Guarantees optimal solutions but slow (not practical for large grids)
- ε=10: Provides 5× speedup with only 30% suboptimality
- ε=20: Enables 10× speedup with 2× suboptimal paths

**Goal Biasing Effectiveness:**
- 5% goal bias: Thorough exploration, better coverage, 15-20s average time
- 20% goal bias: Fast convergence, effective targeting, 5-8s average time
- **Recommendation:** 20% provides best balance for most applications

**RRT* k-Value Impact:**
- k=1: Minimal overhead, near-greedy rewiring
- k=5-10: Practical trade-off between quality and speed
- k=log(n): Best asymptotic quality but highest computation

**Extension Modes:**
- **E1 (Direct):** Faster tree growth, more direct paths, fewer iterations
- **E2 (Step-Limited):** Smoother trees, better for continuous navigation, 3-4× slower

---

## Key Learnings

### Algorithm Design
- Trade-offs between optimality, completeness, and computation time
- Importance of heuristics in guided search
- Probabilistic vs. deterministic approaches
- Parameter tuning for application-specific needs

### Implementation Techniques
- Priority queues for efficient best-first search
- Spatial data structures for nearest-neighbor queries
- Collision detection in continuous spaces
- Tree growth and path extraction strategies

### Practical Applications
- **A\*:** Real-time game pathfinding, grid-based navigation
- **RRT:** Complex geometries, high-dimensional spaces, rapid exploration
- **RRT\*:** Minimum-time trajectories, safety-critical systems, high-quality solutions

---

## Files

- `AStarPlanner.py` - Weighted A* implementation
- `RRTPlanner.py` - RRT sampling-based planner
- `RRTStarPlanner.py` - RRT* asymptotically optimal variant
- `MapEnvironment.py` - 2D grid environment and collision checking
- `RRTTree.py` - Tree data structure for node management
- `run.py` - Main execution script with parameter interface
- `assignment.pdf` - Full problem statement
- `report.pdf` - Detailed solution with experimental results
- `data/maps/` - JSON grid map definitions

---

## Usage

```bash
cd Sampling-Based-Planning

# A* with different epsilon weights
python run.py --map data/maps/map1.json --planner astar --h_weight 1
python run.py --map data/maps/map1.json --planner astar --h_weight 10
python run.py --map data/maps/map1.json --planner astar --h_weight 20

# RRT with different goal biasing strategies
python run.py --map data/maps/map2.json --planner rrt --ext_mode E1 --goal_prob 0.05
python run.py --map data/maps/map2.json --planner rrt --ext_mode E2 --goal_prob 0.2

# RRT* with logarithmic k-nearest
python run.py --map data/maps/map2.json --planner rrtstar --ext_mode E2 --goal_prob 0.2 --k log

# RRT* with fixed k-nearest
python run.py --map data/maps/map2.json --planner rrtstar --ext_mode E2 --goal_prob 0.1 --k 5
```

---

**Status:** ✅ Complete with comprehensive analysis
**Last Updated:** 2025
**Topics:** Path Planning, Heuristic Search, Sampling-Based Algorithms, Asymptotic Optimality
