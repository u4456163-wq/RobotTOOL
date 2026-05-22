# 🤖 RoboTool – URDF-Native Robotics Kinematics Engine

A lightweight robotics kinematics engine for URDF-based robot analysis, geometric Jacobians, and iterative inverse kinematics.

RoboTool focuses on transparent robotics mathematics, lightweight workflows, and direct integration between robot modeling and executable kinematic analysis.

> From URDF → Kinematics → Differential Motion → Simulation-Ready Robotics Workflows

---

# 🚀 Features

## Kinematics

* URDF parsing
* Forward Kinematics (FK)
* Homogeneous transformation propagation
* Full kinematic chain computation
* Support for:

  * revolute joints
  * continuous joints
  * prismatic joints
  * fixed joints

## Differential Robotics

* Geometric Jacobian computation
* Linear and angular Jacobians
* Arbitrary joint-axis support from URDF
* Numerical Jacobian validation using finite differences
* Workspace differential motion analysis

## Inverse Kinematics

* Iterative Inverse Kinematics (IK)
* Jacobian pseudo-inverse solver
* Step-size limiting for numerical stability
* Convergence detection
* Workspace error minimization
* Unreachable target handling

## CLI Tooling

* Lightweight command-line interface
* FK inspection
* Jacobian inspection
* Numerical validation outputs
* Target-based IK solving

---

# 🧠 Motivation

RoboTool originates from a practical limitation encountered during robotics development workflows.

During a biorobotics course, the standard pipeline relied heavily on MATLAB and Simulink. While powerful, these environments introduced significant limitations:

* high resource usage
* licensing restrictions
* export incompatibilities
* limited integration flexibility

A complete 6-DOF robotic arm was successfully modeled and designed, including CAD geometry and kinematic analysis. However, integration into simulation environments became unreliable due to export restrictions and incompatible workflows.

This motivated the exploration of URDF-native robotics pipelines.

After working directly with URDF-based systems, a major gap became evident:

> There is no simple, transparent, and lightweight bridge between robot modeling and executable kinematics.

RoboTool was created to solve that problem.

---

# 🎯 Design Goals

RoboTool emphasizes:

* Explicit robotics mathematics
* Transparent kinematic propagation
* Lightweight workflows
* Reproducibility
* Simulation interoperability
* URDF-native robotics pipelines
* User-controlled modeling and debugging

Instead of hiding robotics concepts behind large frameworks, RoboTool exposes the mathematics directly.

---

# ⚙️ Mathematical Foundations

---

# 1. Forward Kinematics

Unlike traditional Denavit–Hartenberg-only pipelines, RoboTool computes Forward Kinematics directly from the URDF kinematic tree.

This allows:

* arbitrary joint axes
* spatial manipulators
* non-orthogonal configurations
* URDF-native transformations

The pose of each link is propagated using homogeneous transformations.

## Homogeneous Transformation

Each joint transformation is represented as:

```math
T = \begin{bmatrix} R & p \\ 0_{1\times3} & 1 \end{bmatrix}
```

Where:

* (R \in \mathbb{R}^{3\times3}) is the rotation matrix
* (p \in \mathbb{R}^{3}) is the translation vector

---

## Joint Rotation via Rodrigues Formula

To support arbitrary joint axes directly extracted from URDF definitions, RoboTool computes active joint rotations using Rodrigues’ rotation formula.

```math
R_{joint}(\theta)=I_3+\sin(\theta)K+(1-\cos(\theta))K^2
```

Where the skew-symmetric matrix (K) is defined from the unit joint axis vector:

```math
K = \begin{bmatrix} 0 & -u_z & u_y \\ u_z & 0 & -u_x \\ -u_y & u_x & 0 \end{bmatrix}
```

This enables:

* arbitrary spatial rotations
* URDF-native axis handling
* non-DH-compatible robots
* general robotic topologies

---

## Kinematic Chain Propagation

The end-effector pose is propagated along the kinematic chain using sequential homogeneous transformations:

```math
{}^{0}T_n = \prod_{i=1}^{n} {}^{i-1}T_i(q_i)
```

---

# 2. Geometric Jacobian

RoboTool computes the full Geometric Jacobian explicitly.

The Jacobian is separated into:

* linear velocity component (J_v)
* angular velocity component (J_w)

---

## Revolute / Continuous Joints

For rotational joints:

```math
J_i = \begin{bmatrix} z_i \times (p_n - p_i) \\ z_i \end{bmatrix}
```

Where:

* (z_i) is the joint axis expressed in the global frame
* (p_i) is the joint position
* (p_n) is the end-effector position

---

## Prismatic Joints

For prismatic joints:

```math
J_i = \begin{bmatrix} z_i \\ 0_{3\times1} \end{bmatrix}
```

---

## Arbitrary Joint Axis Support

Unlike many simplified robotics pipelines, RoboTool directly respects arbitrary URDF joint axes:

```xml
<axis xyz="ux uy uz"/>
```

Joint axes are projected into the global frame using:

```math
z_i = {}^{0}R_i\hat{u}_i
```

This allows:

* arbitrary robot geometries
* non-standard manipulators
* spatial robot configurations
* direct URDF compatibility

---

# 3. Numerical Jacobian Validation

Analytical Jacobians are internally validated using finite-difference numerical derivatives.

Linear velocity validation:

```math
J_{v_i} \approx \frac{p(q+\delta q_i)-p(q)}{\delta q_i}
```

Typical numerical accuracy:

```text
Max Jv error: 1e-7
Max Jw error: 1e-13
```

This validation guarantees consistency between:

* analytical Jacobians
* FK propagation
* differential motion behavior

---

# 4. Iterative Inverse Kinematics

RoboTool solves Inverse Kinematics numerically using Jacobian pseudo-inverse optimization.

---

## Cartesian Error

The workspace error is defined as:

```math
e = x_d - x_{actual}
```

Where:

* (x_d) is the desired target position
* (x_{actual}) is the current end-effector position

---

## Jacobian Pseudo-Inverse Solver

The joint-space correction is computed using:

```math
\Delta q = J^+ e
```

Where the Moore–Penrose pseudo-inverse is:

```math
J^+ = J^T(JJ^T)^{-1}
```

---

## Stability Constraints

To improve convergence stability near singularities and unreachable targets, RoboTool includes:

* step-size limiting
* angular normalization
* convergence thresholds
* unreachable workspace handling

Joint updates follow:

```math
q_{new}=q_{current}+\alpha\Delta q
```

Where:

* (\alpha) is the damping / learning factor

---

# 🏗️ Architecture

```text
URDF
 └── Parser
      └── Kinematic Graph
           ├── Forward Kinematics
           ├── Jacobian Engine
           ├── Numerical Validation
           └── Iterative IK Solver
```

---

# 📂 Project Structure

```text
robotool/
├── cli.py
├── models.py
├── urdf_parser.py
├── kinematics.py
├── jacobians.py
├── inverse_kinematics.py
└── models_robot/
```

---

# 🚀 CLI Usage

---

# Forward Kinematics

```bash
python3 cli.py Robot_orbitador.urdf
```

Outputs:

* end-effector transformation matrix
* geometric Jacobian
* numerical validation metrics

---

# Inverse Kinematics

```bash
python3 cli.py Robot_orbitador.urdf --target 0.345 0.09 0.395
```

Example output:

```text
Converged in 7 iterations.

Inverse Kinematics Result:
[-0.1676  0.7957  0.3645 -0.0117 -0.3493  0.    ]

Target:         [0.345 0.09  0.395]
Achieved:       [0.345 0.09  0.395]

Position error: 0.000000 m
```

---

# 🧪 Experimental Robot Models

Several URDF models intentionally contain constrained or imperfect kinematic definitions in order to stress-test:

* singular configurations
* malformed kinematic chains
* degenerate Jacobians
* unreachable targets
* limited workspaces
* constrained rotational axes

This allows RoboTool to validate robustness beyond idealized manipulators.

---

# 🔬 Numerical Validation

Analytical Jacobians are validated against numerical finite-difference derivatives.

Typical validation accuracy:

```text
Max Jv error: 1e-7
Max Jw error: 1e-13
```

---

# 📦 Installation

```bash
git clone https://github.com/u4456163-wq/RobotTOOL
cd robotool
pip install -r requirements.txt
```

---

# 🛣️ Roadmap

* [x] URDF Parsing
* [x] Forward Kinematics
* [x] Geometric Jacobians
* [x] Numerical Jacobian Validation
* [x] Iterative Inverse Kinematics

## Planned Features

* [ ] Damped Least Squares IK
* [ ] Orientation-aware IK
* [ ] Null-space optimization
* [ ] Collision-aware IK
* [ ] Dynamics engine
* [ ] Trajectory generation
* [ ] URDF tree support
* [ ] ROS2 bridge
* [ ] Visualization backend

---

# 🧩 Design Philosophy

RoboTool is built around a simple principle:

> Robotics tools should expose the mathematics, not hide them.

The engine prioritizes:

* transparency
* explicit kinematic modeling
* reproducibility
* lightweight robotics workflows
* interoperability with custom robotics pipelines

Rather than abstracting robotics behind heavyweight frameworks, RoboTool exposes:

* transformations
* Jacobians
* kinematic propagation
* numerical IK behavior
* workspace dynamics

Directly to the user.

---

# 🤝 Credits

Inspired by:

* URDF-based robotics workflows
* geometric robotics
* differential kinematics
* modern robotics pipelines
* robotics simulation toolchains

Built as part of the RoboTool / RobotCAD ecosystem.
