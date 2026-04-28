# Reduced-Order Gait Design and Full-Order Nonlinear Control of a Three-Link Planar Biped

This repository contains our implementation of a **three-link planar biped** modeled and controlled using the **Hybrid Zero Dynamics (HZD)** framework. The project combines:

- dynamic modeling of a planar underactuated biped,
- reduced-order gait design using zero dynamics,
- optimization of gait parameters using Bézier virtual constraints,
- full-order nonlinear simulation with feedback linearization and PD stabilization,
- and stability evaluation using phase portraits and Poincaré analysis.

The main idea is to first design a walking gait on a **reduced-order zero-dynamics manifold**, and then apply that gait to the **full nonlinear hybrid model** to generate walking in the complete six-state system.

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

- $$ q_1^- $$
  $$ \dot{q}_1^- $$
  are the pre-impact reduced states,
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
The reduced-order zero-dynamics model yields a **periodic gait**, visible through a closed and bounded phase portrait in the \((q_1,\dot{q}_1)\) plane.

### Full-Order Walking
When the optimized gait is applied to the full nonlinear model:

- the robot produces a clear **multi-step walking motion**,
- the joint angles and velocities remain bounded,
- the controller tracks the virtual constraints closely,
- and the full-order phase portrait shows convergence toward a repeating orbit.

### Stability Analysis
Stability is evaluated using:

- **phase portraits**
- **Poincaré analysis**

The post-impact state sequence converges toward a fixed point, and the Poincaré error decreases to near zero within roughly 10 steps. In extended simulations, the gait remains bounded and repeats consistently.

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
