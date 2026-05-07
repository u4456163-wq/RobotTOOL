# RoboTooL
# 🤖 RobotCAD – Kinematic Analysis Tool

A lightweight toolchain for robotic modeling and forward kinematics, designed to bridge the gap between URDF design and simulation-ready transformations.

> From URDF → Transformation Matrices → Simulation-ready pipeline

---

## 🚀 Demo

```bash
python3 cli.py robot_raw.urdf
```

Example output:

```
[[ 0.00  1.00  0.00  0.345 ]
 [ ...  ...  ...  ...   ]]
```

---

## ⚙️ Features

* URDF parsing
* Forward kinematics (Denavit–Hartenberg based)
* Homogeneous transformations
* Modular CLI interface
* Simulation-ready outputs

---

## 🧠 Motivation

This project originates from a real limitation in robotics workflows.

During a biorobotics course, the standard pipeline relied on MATLAB and Simulink. While powerful, these tools introduced practical constraints such as high resource usage and licensing limitations.

A full 6-DOF robotic arm was successfully developed — including kinematic modeling and CAD design. However, integration into simulation environments failed due to export restrictions and incompatible formats.

This led to the exploration of URDF-based workflows, where RobotCAD became a key turning point.

After resolving initial setup issues and working directly with the tool, a clear gap became evident:

> There is no simple, transparent, and lightweight bridge between robot modeling and executable kinematics.

**RobotCAD focuses on:**

* Transparency (explicit math & transformations)
* Control (user-driven modeling)
* Lightweight workflows
* Direct integration with custom pipelines

What started as a limitation became the foundation for building a better workflow.

---

## 🏗️ Architecture

```
URDF → Parser → DH Parameters → Transformation Matrices → Output
```

**Modules:**

* `parser/`
* `kinematics/`
* `cli/`

---

## 📐 Example

**Input:**

* 6-DOF robotic arm (URDF)

**Output:**

* End-effector pose
* Transformation matrix per joint

---

## 🎯 Use Cases

This tool is designed for:

* Extracting homogeneous transformation matrices from URDF-defined robot poses
* Verifying robot kinematic chains before simulation 
* Debugging URDF exports and robot configurations
* Lightweight forward kinematics analysis 
* Educational and research workflows 
* Integration into custom robotics pipelines 

---

## 📦 Installation

```bash
git clone https://github.com/u4456163-wq/RobotTOOL
cd robotool
pip install -r requirements.txt
```

---

## 🛣️ Roadmap

* [x] Forward Kinematics
* [ ] Inverse Kinematics
* [ ] ROS2 integration
* [ ] Simulation visualization

---

## 🔬 Design Philosophy

RobotCAD is built around a simple principle:

> Robotics tools should expose the math, not hide it.

Instead of abstracting away transformations, this toolchain emphasizes:

* Explicit kinematic modeling
* Reproducibility
* Integration with custom workflows (CAD, URDF, simulation)

---

## 🤝 Credits

Inspired by robotics toolchains and Denavit–Hartenberg-based modeling.

Built as part of the RobotCAD ecosystem.
