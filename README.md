# Reduced-Order Gait Design and Full-Order Nonlinear Control of a Three-Link Planar Biped

This repository contains our implementation of a **three-link planar biped** modeled and controlled using the **Hybrid Zero Dynamics (HZD)** framework. The project combines:

- dynamic modeling of a planar underactuated biped,
- reduced-order gait design using zero dynamics,
- optimization of gait parameters using Bézier virtual constraints,
- full-order nonlinear simulation with feedback linearization and PD stabilization,
- and stability evaluation using phase portraits and Poincaré analysis.

The main idea is to first design a walking gait on a **reduced-order zero-dynamics manifold**, and then apply that gait to the **full nonlinear hybrid model** to generate walking in the complete six-state system.

---

## Table of Contents

- [Project Overview](#project-overview)
- [Robot Model](#robot-model)
- [Methodology](#methodology)
  - [1. Dynamic Modeling](#1-dynamic-modeling)
  - [2. Zero Dynamics and Virtual Constraints](#2-zero-dynamics-and-virtual-constraints)
  - [3. Optimization-Based Gait Design](#3-optimization-based-gait-design)
  - [4. Full-Order Nonlinear Control](#4-full-order-nonlinear-control)
- [Results](#results)
  - [Reduced-Order Gait](#reduced-order-gait)
  - [Full-Order Walking](#full-order-walking)
  - [Stability Analysis](#stability-analysis)
- [Running the Project](#running-the-project)
- [Note on Ground Drift / Sinking](#note-on-ground-drift--sinking)

---

## Project Overview

Bipedal locomotion is difficult because walking is:

- **nonlinear**,
- **underactuated**,
- and **hybrid**, with continuous swing dynamics and discrete impact events.

To make the problem tractable, this project uses **Hybrid Zero Dynamics**, where the actuated joints are constrained to follow desired trajectories defined by **Bézier polynomials**. These virtual constraints reduce the gait-design problem to a smaller set of states, making it possible to search for periodic walking motions more efficiently.

After obtaining a reduced-order gait, the project evaluates whether that gait can be realized by the **full-order robot model** under a nonlinear controller.

---

## Robot Model

The robot is a **three-link planar biped** made of:

- a **stance leg**,
- a **swing leg**,
- and a **torso**.

The generalized coordinates are:

- `q1` — absolute stance-leg angle,
- `q2` — swing-leg angle relative to the stance leg,
- `q3` — torso angle relative to the stance leg.

Key assumptions:

- motion is restricted to the **sagittal plane**,
- the robot has **point feet**,
- only one foot is in contact with the ground during single support,
- the stance foot does not slip,
- impact at foot strike is modeled as **instantaneous**.

Because the stance foot is passive, the robot is **underactuated**.

<img width="497" height="552" alt="3_Link_Model" src="https://github.com/user-attachments/assets/6fa77573-3130-4468-8f40-78b13641922e" />

---

## Methodology

### 1. Dynamic Modeling
The robot dynamics are derived using:

- forward kinematics,
- kinetic and potential energy,
- Euler–Lagrange equations,
- manipulator-form dynamics.

The continuous swing-phase model is written in control-affine form:

$$
\dot{x} = f(x) + g(x)u
$$

where the full state is:

$$
x = [q_1,\ q_2,\ q_3,\ \dot{q}_1,\ \dot{q}_2,\ \dot{q}_3]^T
$$

An **impact map** is also implemented to model the discrete velocity reset that occurs when the swing foot strikes the ground.

---

### 2. Zero Dynamics and Virtual Constraints
To design the gait, the actuated coordinates are constrained using **virtual constraints**:

$$
y =
\begin{bmatrix}
q_2 - b_2(s) \\
q_3 - b_3(s)
\end{bmatrix}
$$

where `b2(s)` and `b3(s)` are **fourth-order Bézier polynomials** and `s` is a normalized gait timing variable computed from `q1`.

When the outputs satisfy `y = 0`, the robot evolves on the **zero-dynamics manifold**, reducing the walking dynamics to the cyclic variable:

$$
z = [q_1,\ \dot{q}_1]^T
$$

This reduced-order model is used for gait design and optimization.

---

### 3. Optimization-Based Gait Design
The gait is parameterized by the vector:

$$
f = [q_1^-,\ \dot{q}_1^-,\ \alpha_{2,3},\ \alpha_{2,4},\ \alpha_{2,5},\ \alpha_{3,3},\ \alpha_{3,4},\ \alpha_{3,5}]
$$

where:

- $(q_1^-,\dot{q}_1^-)$ are the pre-impact reduced states,
- the remaining values are selected Bézier coefficients for the swing leg and torso.

The optimizer:

- simulates one step of the zero dynamics,
- enforces **periodicity** through a nonlinear equality constraint,
- and minimizes a control-effort cost.

This produces a periodic reduced-order gait candidate.

---

### 4. Full-Order Nonlinear Control
The optimized gait is then applied to the **full nonlinear hybrid model**.

A **feedback-linearizing controller** is used to regulate the virtual-constraint outputs. The output dynamics are written using Lie derivatives:

$$
\ddot{y} = L_f^2 h(x) + L_gL_fh(x)\,u
$$

and the control input is chosen as:

$$
u = (L_gL_fh)^{-1}(v - L_f^2h)
$$

with a PD stabilizing term:

$$
v = -K_p y - K_d \dot{y}
$$

This drives the full-order system toward the desired gait manifold during swing.

---

## Results

### Reduced-Order Gait
The reduced-order zero-dynamics model yields a **periodic gait**, visible through a closed and bounded phase portrait in the $(q_1,\dot{q}_1)$ plane.


<img width="1439" height="930" alt="MP3_1" src="https://github.com/user-attachments/assets/e1177a80-4aca-4de0-8ba2-6eb2e253172e" />

---

### Full-Order Walking

When the optimized gait is applied to the full nonlinear model:

- the robot produces a clear **multi-step walking motion**,
- the joint angles and velocities remain bounded,
- the controller tracks the virtual constraints closely,
- and the full-order phase portrait shows convergence toward a repeating orbit.

<img width="495" height="1599" alt="MP4_4" src="https://github.com/user-attachments/assets/3c0862a2-b031-4d7b-a0a5-ec87e515b1b3" />

---

### Stability Analysis
Stability is evaluated using:

- **phase portraits**

The full-order phase portrait of $(q_1,\dot{q}_1)$ shows repeated walking cycles over multiple steps. After a short transient, the trajectories converge toward a compact inner loop, indicating stable periodic behavior. The sharp diagonal jumps correspond to the hybrid impact resets at footstrike.


<img width="1120" height="840" alt="MP4_1" src="https://github.com/user-attachments/assets/3711be04-a605-4c37-8466-7d3ee85d4852" />

---
- **Joint angles and Angular velocities**

The joint trajectories remain bounded and repeat consistently across steps. The stance leg, swing leg, and torso follow a clear walking pattern, while the joint velocities show periodic oscillations with resets at impact. This indicates that the controller is able to generate sustained and physically realistic walking.

<img width="1120" height="840" alt="MP4_2" src="https://github.com/user-attachments/assets/8f943aa5-c0ec-410c-a37a-a17d94e5593d" />

---
- **Poincaré analysis**

Step-to-step stability is evaluated using the post-impact state $(q_1^+,\dot{q}_1^+)$. The Poincaré error decreases rapidly and approaches zero within roughly 10 steps, showing convergence to a fixed point. This confirms that the closed-loop gait is asymptotically stable in the full-order hybrid model.

<img width="2437" height="1191" alt="MP4_3" src="https://github.com/user-attachments/assets/2a01a471-df43-4edd-90bb-30ab6d83f854" />

---

## Running the Project

The project workflow is:

1. **Generate the symbolic dynamics functions**
2. **Optimize the gait parameters**
3. **Run the full-order walking simulation**

### Generate the model files (loads the model files to the \autogen)
Run:

```matlab
generate_functions
```
### Run the optimizer
Run:

```matlab
Optimize
```
### Run the walking simulation with the optimized 'f' from the optimizer
Run:

```matlab
sim_and_plot_full_dynamics
```

## Note on Ground Drift / Sinking

A small downward drift in absolute ground height may be visible over long simulations. In this implementation, this is treated as an **initialization artifact** caused by starting from a non-upright stance configuration rather than exactly resetting the stance foot to ground level after each impact.

This effect does **not** change the step-to-step stability conclusion, since the closed-loop gait still converges in the reduced post-impact state space.

---
