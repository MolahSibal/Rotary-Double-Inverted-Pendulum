# Rotary Double Inverted Pendulum

Modeling, simulation, and control of a **Quanser Rotary Double Inverted Pendulum (DBIP)** developed during my summer research internship at the University of Waterloo.

The objective of this project was to investigate how control theory can be used to stabilize an inherently unstable and underactuated system: two inverted pendulum links controlled through a single rotary actuator.

## Project Overview

The project follows the complete control-system development process:

**First-principles modeling → Linearization → Controller design → Simulation → Hardware implementation**

The nonlinear equations of motion were derived using **Lagrangian mechanics**, accounting for the kinetic (translational & rotational) and potential energy of the system, joint damping, and DC motor dynamics.

The nonlinear model was then linearized about the upright equilibrium and represented in state-space form:

$$
\dot{x} = Ax + Bu
$$

where the state vector is

$$
x =
\begin{bmatrix}
\theta &
\alpha &
\phi &
\dot{\theta} &
\dot{\alpha} &
\dot{\phi}
\end{bmatrix}^{T}.
$$

## Control Design

Several control and state-estimation techniques were investigated, including:

- Linear Quadratic Regulator (LQR)
- Pole-placement state feedback
- Full-order observers
- Minimum-order observers
- Reference tracking and integral action

Controllers were first simulated on the linear model of the plant then deployed on the real hardware to verify their performance. A nonlinear model "DBIP_NonlinearModel_StateFeedback.m" is availble in the Simulation-Software folder.

## Results

The final controller successfully stabilized both pendulum links while allowing the rotary arm to track a commanded position.

A significant part of the project also involved validating the mathematical model. Inconsistencies identified between the provided model, documentation, and implementation motivated an independent derivation of the system dynamics from first principles.

## Repository Structure

```text
Rotary-Double-Inverted-Pendulum/
├── Simulation-Software/
├── HIL_Software/
├── Documentation/
└── README.md
```
## Author

**Peter Jang**  

B.Eng. Aerospace Engineering — Avionics

Research conducted at the University of Waterloo
##
