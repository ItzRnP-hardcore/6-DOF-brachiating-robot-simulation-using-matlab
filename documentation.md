# Technical Documentation
## Interactive 6-DOF Brachiating Robot Simulation

**Author:** Rudranarayan — Mechanical Engineering, IIT Kharagpur  
**Language:** MATLAB (single-file, no toolboxes required)  
**File:** `interactive_6dof_robot.m`

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [System Architecture](#2-system-architecture)
3. [Kinematic Model](#3-kinematic-model)
4. [Inverse Kinematics Solver](#4-inverse-kinematics-solver)
5. [RRT* Path Planner](#5-rrt-path-planner)
6. [Global Routing (A*)](#6-global-routing-a)
7. [Collision Detection](#7-collision-detection)
8. [Brachiating Locomotion](#8-brachiating-locomotion)
9. [GUI and Controls](#9-gui-and-controls)
10. [Environment Configuration](#10-environment-configuration)
11. [Known Limitations and Tuning](#11-known-limitations-and-tuning)

---

## 1. Project Overview

This simulation models a **6-DOF serial robot arm** that locomotes across a planar **hexagonal docking grid** using a brachiating strategy — alternating its base and end-effector between fixed docking points, similar to how a gibbon swings between branches.

The system integrates:
- **Forward & Inverse Kinematics** using Craig's DH convention
- **RRT\*** joint-space path planning with L6-norm metric
- **A\*** graph search for global multi-hop routing
- **Full collision detection** (sphere, capsule, box, self, ground)
- **Dynamic obstacle placement** via a MATLAB GUI

---

## 2. System Architecture

The entire simulation lives in a single MATLAB function with nested subfunctions. State is shared via closures over the outer function's workspace.

### Key State Variables

| Variable | Type | Description |
|---|---|---|
| `Global_Base_T` | 4×4 matrix | Homogeneous transform of the robot's current base joint in world frame |
| `current_base_joint` | int (1 or 6) | Which joint is currently latched as the base |
| `current_thetas` | 1×6 double | Current joint angles in degrees |
| `planned_path_q` | N×6 double | RRT* path waypoints (joint space) |
| `docking_points` | M×3 double | World-frame XYZ positions of all hexagonal grid nodes |
| `obstacles` | struct array | User-added dynamic obstacles (type, pos, size) |
| `ee_trace` | 3×N double | End-effector position history for trail rendering |

---

## 3. Kinematic Model

### DH Parameters (Craig's Convention)

The robot uses **Craig's modified DH convention**, where the transform for joint *i* is:

```
T_i = Rot_x(alpha_{i-1}) * Trans_x(a_{i-1}) * Rot_z(theta_i) * Trans_z(d_i)
```

Implemented as:

```matlab
function T_i = compute_Ti(a, alpha, d, theta)
    Q = [1, 0, 0, a; 0, cosd(alpha), -sind(alpha), 0;
         0, sind(alpha), cosd(alpha), 0; 0, 0, 0, 1];
    R = [cosd(theta), -sind(theta), 0, 0; sind(theta), cosd(theta), 0, 0;
         0, 0, 1, d; 0, 0, 0, 1];
    T_i = Q * R;
end
```

### DH Table

| Joint | a (mm) | α (°) | d (mm) | θ (variable) |
|-------|--------|-------|--------|--------------|
| 1 | 0 | 0 | 10 | θ₁ |
| 2 | 0 | −90 | 10 | θ₂ |
| 3 | 10 | 0 | 0 | θ₃ |
| 4 | 0 | −90 | 10 | θ₄ |
| 5 | 0 | +90 | 10 | θ₅ |
| 6 | 0 | −90 | 10 | θ₆ |

### Forward Kinematics

`getForwardKinematics(thetas)` computes all 7 frame positions (base + 6 joints) in the world frame, accounting for which joint is currently the base:

- **Base joint = 1:** `T_global_1 = Global_Base_T`, chain runs forward.
- **Base joint = 6:** `T_global_1 = Global_Base_T / T_local_6`, effectively reversing the kinematic chain.

Returns `positions` (3×7 array of joint XYZ) and `T_EE` (4×4 end-effector transform).

---

## 4. Inverse Kinematics Solver

`solveIK(target_pos, q_seed, target_z_dir, ori_weight)` uses **numerical optimisation** via MATLAB's `fminsearch`.

### Cost Function

```
cost = 10 * ||p_ee - p_target|| + ori_weight * ||z_ee - z_target||
```

where `z_ee` is the end-effector's Z-axis direction and `z_target` is the desired tool orientation.

### Multi-Seed Strategy

1. Starts with `q_seed` (current configuration).
2. Adds 20 random seeds uniformly sampled from [−180°, +180°]⁶.
3. Runs `fminsearch` from each seed (1500 iterations, 3000 function evaluations).
4. Tracks both:
   - `best_q`: best collision-free solution with `fval < 2.0`
   - `best_q_any`: best solution overall (used as fallback if no collision-free solution found)
5. **Success criterion:** `fval < 10.0`

### Orientation Control

When "Force Final Orientation" is enabled, the target Z-axis is computed as:

```matlab
target_z_dir = rotz(rz) * roty(ry) * rotx(rx) * [0;0;1];
```

Default orientation is `[0;0;−1]` (pointing down) for base-joint-1 mode.

---

## 5. RRT* Path Planner

`planRRTStar(q_start, q_goal, max_nodes, step_size, goal_bias)` implements RRT* in **joint space** using the **L6 norm** as the distance metric.

### Why L6 Norm?

The L6 norm `||q||₆ = (Σ |qᵢ|⁶)^(1/6)` heavily penalises any single large joint excursion. This biases the planner toward paths where all joints move roughly equally, avoiding large rotations in individual joints that can cause kinematic singularities or awkward poses.

### Algorithm

```
1. Initialise tree with q_start
2. For each iteration:
   a. Sample q_rand (goal with probability goal_bias, else random)
   b. Find nearest node by L6 distance
   c. Steer toward q_rand by step_size in L6 metric
   d. Collision check q_new
   e. Find all nodes within search_radius (= 2.5 × step_size)
   f. Choose best parent (lowest cost-to-come + edge cost)
   g. Add node to tree
   h. Rewire: update parent of nearby nodes if cheaper through q_new
   i. Check if q_new is within step_size of q_goal → extract path
```

### Angular Wrapping

All angle arithmetic uses modular wrapping to handle the ±180° discontinuity:

```matlab
diff = mod(q2 - q1 + 180, 360) - 180
```

This ensures distances are always computed over the shorter arc.

---

## 6. Global Routing (A*)

`findGlobalRoute(start_idx, goal_idx, blocked_edges)` runs **A\*** over the docking point graph.

### Graph Structure

- **Nodes:** All docking points in the hexagonal grid.
- **Edges:** Any two nodes within `max_reach = 35` units of each other.
- **Heuristic:** Euclidean distance to goal.
- **Blocked edges:** Edges that failed RRT* planning are added to `blocked_edges` and excluded from future searches.

### Dynamic Replanning

In `planAndExecuteDynamicMultiHop()`, when a hop fails:

1. The failed edge `(curr_dock, next_dock)` is appended to `blocked_edges`.
2. A\* reruns from `curr_dock` to `terminal_dock_idx` with the updated blocked set.
3. If no alternate route exists, the planner reports failure.

---

## 7. Collision Detection

`checkCollisions(thetas)` runs five checks in order, returning `true` on the first hit.

### Check 1: Predefined Sphere Obstacles

For each sphere (centre + radius) and each robot link (modelled as a capsule):

```
dist(sphere_centre, link_segment) < sphere_radius + link_radius + 0.5
```

Uses `dist3D_Point_to_Segment()`.

### Check 2: Predefined Capsule Obstacles

Capsule–capsule distance check using `dist3D_Segment_to_Segment()` (GJK-style closest-point on two line segments).

### Check 3: Dynamic (User-Added) Obstacles

- **Sphere:** Point-to-segment distance threshold.
- **Cylinder:** Horizontal (XY-plane) distance from link start point to cylinder axis — a simplified vertical-cylinder check.
- **Box (AABB):** Full `dist3D_Segment_to_AABB()` check — analytically finds the minimum distance from a line segment to an axis-aligned bounding box by evaluating at segment endpoints and all slab face intersections.

### Check 4: Self-Collision

Checks non-adjacent link pairs: `[1,4], [1,5], [1,6], [2,5], [2,6], [3,6]` with a `self_collision_margin = 1.0` buffer.

### Check 5: Ground Plane

```matlab
if any(positions(3,:) < -0.01)
    isCollision = true;
end
```

Prevents any joint centre from penetrating the ground plane (Z = 0).

---

## 8. Brachiating Locomotion

### `swapBase()`

This is the core brachiating operation. It recomputes `Global_Base_T` so that the new base is at the current end-effector position:

**When current base is Joint 1 → swap to Joint 6:**
```matlab
Global_Base_T = Global_Base_T * T_local_6;
current_base_joint = 6;
```

**When current base is Joint 6 → swap to Joint 1:**
```matlab
Global_Base_T = Global_Base_T / T_local_6;
current_base_joint = 1;
```

After swapping, `Global_Base_T(3,4) = 0` clamps the new base Z to the ground plane (docking point height).

### Multi-Hop Sequence

```
1. Identify terminal dock: nearest dock from which IK can reach the target
2. Compute global A* route: current_dock → terminal_dock
3. For each hop:
   a. Solve IK for end-effector to reach next_dock
   b. Plan RRT* path
   c. Execute path (animate)
   d. Call swapBase()
4. From terminal dock, solve IK + RRT* for final target position
```

---

## 9. GUI and Controls

The GUI is built entirely with MATLAB `uicontrol` elements. All callbacks use closures to access and modify the shared state.

### Layout

```
Left panel (35% width): all controls
Right panel (60% width): 3D axes
```

### Button Callbacks

| Button | Callback | Description |
|---|---|---|
| PLAN SINGLE HOP | `planSingleHopAction()` | IK + RRT* for one move; plots path in green |
| EXECUTE MOTION | `executeMotionAction()` | Animates `planned_path_q` step by step |
| Dynamic Plan to Target | `planAndExecuteDynamicMultiHop()` | Full brachiating sequence |
| LATCH & SWAP | `swapBase()` | Manual base-flip |
| SHOW FREE WS | `plotFreeWorkspace()` | 5000-sample workspace scatter |
| ADD OBSTACLE | `addObstacle()` | Reads UI fields; appends to `obstacles` struct |
| CLEAR OBSTACLES | `clearObstacles()` | Empties `obstacles` struct |

---

## 10. Environment Configuration

### Hexagonal Docking Grid

Generated with a row-offset hex pattern, step size 30 units:

```matlab
row_height = step_size * (sqrt(3)/2);
for row = -3:3
    for col = -3:3
        x = col * step_size;
        y = row * row_height;
        if mod(row, 2) ~= 0, x = x + (step_size / 2); end
        % keep if |x|<=65 and |y|<=65
    end
end
```

### Predefined Static Obstacles

**Spheres** (`env_spheres` — columns: X, Y, Z, radius):

| X | Y | Z | R |
|---|---|---|---|
| 35 | 15 | 20 | 7 |
| −40 | −40 | 20 | 8 |
| 45 | −30 | 25 | 6 |

**Capsules** (`env_capsules` — columns: X1,Y1,Z1, X2,Y2,Z2, radius):

| Start | End | R |
|---|---|---|
| (−25, 20, 0) | (−25, 20, 45) | 4 |
| (15, −25, 0) | (15, −25, 30) | 5 |
| (−10, −10, 30) | (20, −10, 30) | 3 |
| (0, 20, 15) | (20, 40, 25) | 3 |

### Link Radii

```matlab
cyl_radius = [2.5, 3, 3, 3, 3, 2.5];  % one per link
```

---

## 11. Known Limitations and Tuning

### IK Success Rate

The IK solver uses `fminsearch` which is gradient-free and may stall in local minima. Increasing seeds from 20 to 40+ improves reliability for highly constrained targets. The orientation weight (`ori_weight = 30`) can be reduced to prioritise position accuracy over orientation matching.

### RRT* Performance

With `max_nodes = 1500` and `step_size = 15°`, planning typically completes in 0.5–3 seconds in MATLAB. For denser obstacle environments, increase `max_nodes` to 2500–4000.

### Ground Plane Constraint

The `Z < -0.01` ground check uses a small epsilon to tolerate floating-point dock intersections. If the robot frequently fails hops that look feasible visually, this threshold can be relaxed to `-0.5`.

### Cylinder Obstacle Check (Simplified)

The dynamic cylinder obstacle check uses only the 2D XY distance to the cylinder axis — it does not check Z extent. This means a cylinder placed at Z=10 still blocks links that pass in XY-proximity at Z=0. For improved accuracy, replace with a full 3D capsule check using `dist3D_Segment_to_Segment()`.

### Self-Collision Margin

`self_collision_margin = 1.0` adds a 1-unit buffer to all self-collision checks. This can be reduced to `0.5` if the planner is rejecting too many valid configurations.

---

*Documentation generated for `interactive_6dof_robot.m` — IIT Kharagpur, Mechanical Engineering.*
