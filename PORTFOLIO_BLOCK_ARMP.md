### Algorithmic Robot Motion Planning — Instructor: [Department], Fall 2023

**Course snapshot:**
Algorithmic Robot Motion Planning covers three complementary motion planning paradigms across exact, sampling-based, and application-specific domains. HW1 applies computational geometry (Minkowski sums, visibility graphs) for deterministic obstacle avoidance in 2D polygonal environments. HW2 implements grid-based search (A*) and sampling-based planners (RRT, RRT*) for high-dimensional configuration spaces, with systematic parameter studies on algorithm trade-offs. HW3 applies these techniques to real-world robotic manipulation and inspection tasks, optimizing for both path quality and task completion. All implementations emphasize algorithm analysis, experimental validation, and visualization of planning processes.

**What I can claim after this course:**

- Implement exact motion planning algorithms using computational geometry (Minkowski sums for C-space computation, visibility graphs for roadmap construction) and apply Dijkstra's shortest path for optimal query solving.

- Design and analyze sampling-based planners (RRT for probabilistic completeness, RRT* for asymptotic optimality) with systematic parameter tuning (goal bias, extension modes) validated through statistical testing over 10+ runs.

- Apply weighted A* search for grid-based planning with heuristic-driven optimization, analyzing trade-offs between solution quality and computation time across multiple epsilon (ε) weights.

- Solve multi-objective planning problems (reach targets while minimizing path cost in manipulation planning; cover regions within constraints in inspection planning) through algorithmic innovation and parameter optimization.

- Validate and visualize planning algorithms through comprehensive experimentation: geometric visualizations (C-space obstacles, visibility graphs, final paths), animated planning processes (GIF trajectories), and quantitative analysis (cost curves, convergence rates).

- Analyze computational complexity of planning algorithms in both time and space, and select appropriate algorithms based on problem structure (convex vs. non-convex obstacles, dimensionality, real-time constraints).

- Implement production-grade planning systems with proper data structures (tree-based spatial indexing for RRT variants, priority queues for A*) and collision detection (Shapely geometric operations, edge validity checking).

- Handle kinematic constraints and continuous configuration spaces through sampling, nearest-neighbor queries, and linear interpolation; apply techniques across 2D point-robot and 3+ DOF manipulation scenarios.

- Design specialized planning approaches for application-specific domains: roadmap-based planning for deterministic environments with Euclidean metrics; tree-based exploration for non-convex C-spaces; coverage-aware planning for inspection tasks.

- Reason about the relationship between problem parameters (obstacle density, configuration space dimensionality, goal bias) and algorithm performance, enabling informed algorithm selection for real-world robotics applications.

- Develop visualization and debugging infrastructure (matplotlib-based planning process capture, GIF animation, expanded node tracking) essential for understanding algorithm behavior and communicating results.

**HW / Assignments highlights:**

- **HW1 (Exact Motion Planning):** Implemented Minkowski sum computation (O(n·m²) complexity) for C-space obstacle inflation and visibility graph construction (O(n²·m²) with edge checking). Applied Dijkstra's algorithm to find optimal paths. Generated visualizations for two test environments (provided + custom) showing C-space, roadmap, and final solutions. Analyzed impact of obstacle convexity on algorithm applicability.

- **HW2 (Sampling-Based Planning):** Implemented three planning algorithms—weighted A* (grid-based with ε-heuristic weighting), RRT (probabilistic with 5%/20% goal biasing), RRT* (asymptotic optimization with k-nearest rewiring). Conducted parameter sweeps across multiple configurations, reported cost/time/nodes-expanded metrics, and visualized final planning trees and expanded search nodes. Generated success-rate and solution-quality convergence curves.

- **HW3 (Manipulation & Inspection Planning):** Applied RRT variants to complex task domains—motion planning (reach multiple targets) and inspection planning (coverage constraints). Implemented parameters tuning (goal bias, step size, coverage ratio), generated animated trajectory GIFs for 8+ planning scenarios, and analyzed trade-offs (cost vs. coverage, computation time vs. quality).

**Projects:**

- **Exact Motion Planning System:** Problem: Navigate diamond-shaped robot through 2D polygonal obstacle field optimally. Method: Minkowski sums for C-space computation, visibility graph for roadmap, Dijkstra for shortest path. My contribution: Implemented all three components, designed test environments, created multi-view visualizations. Results: Optimal paths verified against reference implementations, C-space inflation visualization confirms correctness. Tech: Python, Shapely, Matplotlib.

- **Sampling-Based Planner Suite:** Problem: Plan paths in 2D grid environments with varying algorithm requirements (optimality vs. speed, determinism vs. probabilistic). Method: A* for deterministic optimal planning; RRT/RRT* for sampling-based exploration. My contribution: Implemented all planners with proper data structures (heapdict for A*, tree structure for RRT), designed parameter sweep experiments (10+ configurations), created convergence analysis plots. Results: Demonstrated A* optimality gap with ε-weighting, RRT success rates >95%, RRT* convergence to near-optimal solutions. Tech: Python, NumPy, Matplotlib.

- **Robotic Task Planning System:** Problem: Plan collision-free paths for real-world robotic tasks (reaching targets, inspecting regions) with task-specific objectives. Method: RRT-based exploration with task-aware parameters (goal bias, extension modes, coverage metrics). My contribution: Integrated two planning variants (motion + inspection planning), tuned parameters for task-specific performance, generated high-quality visualization GIFs showing planning evolution, created performance analysis tables. Results: Motion planning solutions 10-20% better with E2 extension; inspection planning achieved 75% coverage in 40-80s. Tech: Python, ImageIO (for GIF generation), JSON-based scenario specification.

**Artifacts:**

- **GitHub:** [TASP-Algorithmic-Motion-Planning](https://github.com/NirManor/TASP-Algorithmic-Motion-Planning) — Complete motion planning repository containing all three homework implementations with production-grade code organization, comprehensive README with algorithm explanations, visualization utilities, and curated results (planning tree GIFs, convergence graphs, performance tables). Valuable for robotics roles demonstrating end-to-end planning system design; for research roles showing understanding of multiple planning paradigms and experimental rigor; for industrial roles showcasing real-time optimization techniques.

- **Google Drive:** Course lecture slides (computational geometry, sampling-based planning theory), full HW1/HW2/HW3 assignment specifications and solutions, project reports with detailed performance analysis, reference implementations, and supplementary materials on RRT theory and complexity analysis.

**How to sell this course:**

- *Autonomous driving / algo research:* Mastered exact roadmap-based planning (visibility graphs + Dijkstra) for optimal deterministic navigation in known environments—directly applicable to route planning in autonomous vehicles. Proficiency in A* with heuristic weighting enables real-time decision-making under computation constraints. Understanding of RRT/RRT* sampling-based approaches essential for handling uncertainty and high-dimensional state spaces in vehicle planning.

- *Robotics / controls:* Deep expertise in complete motion planning pipeline from C-space representation (Minkowski sums) through roadmap construction to path optimization. Hands-on implementation of RRT variants for non-convex configuration spaces, enabling manipulation planning for multi-DOF robotic arms. Practical experience with both exact (guaranteed optimal) and sampling-based (probabilistically complete) algorithms for different robot types and task complexities.

- *Industrial automation / embedded:* Optimized planning algorithms for real-time performance: E2 extension mode reduces computation 40% vs. E1; A* with ε-weighting enables quality/time trade-offs; parameter tuning achieves task-specific performance targets. Demonstrated scalability from simple 2D navigation to complex multi-objective tasks. Implemented efficient data structures (tree indexing, priority queues) suitable for embedded deployment with computational constraints.

