---
title: Lab 8
usemathjax: true
---

<script type="text/javascript" id="MathJax-script" async
  src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js">
</script>

# Linearization of non-linear dynamics around fixed points

Find fixed points of the following systems and linearize their dynamics around these fixed points, i.e.:

- given a system \\( \dot{x} = f(x) \\), find \\(\overline{x}\\) such that \\(f(\overline{x})=0\\) and 
- formulate approximate dynamics \\(\dot{\Delta x} = A ⋅ \Delta x\\), where \\(A\\) is the matrix you need to find and \\(\Delta x=x-\overline{x}\\).

[Notes with solutions and hints](linearization-solutions.pdf).

## System 1

Consider the following 1-dimensional system that can be used to model population growth. Here, \\(x\\) is the population size, and \\(P_{max}\\) is the population limit above which resources become scarce.

$$\dot{x} = f(x) = x(P_{max}-x)$$

## System 2

Damped pendulum (\\(\delta\\) is the damping coefficient), where \\(\theta\\) denotes the angle.

$$\ddot{\theta} = -\sin(\theta) - \delta\dot{\theta}$$

Denote

$$x = \begin{pmatrix} x_1 \\ x_2\end{pmatrix} = \begin{pmatrix} \theta\\ \dot{\theta} 
\end{pmatrix} $$


$$\dot{x} =  \begin{pmatrix}\dot{x_1} \\ \dot{x_2}\end{pmatrix} = \begin{pmatrix}x_2 \\ -\sin(x_1) - \delta x_2 \end{pmatrix}
$$ 

which is non-linear due to the \\(\sin\\) function.

## System 3

In the following system you can assume \\(-\pi \le \theta \le \pi\\).


$$\dot{r} = r^2 - r$$

$$\dot{\theta} = \sin^2(\theta / 2)$$


## System 4

$$\begin{pmatrix}\dot{x} \\ \dot{y}\end{pmatrix} = 
\begin{pmatrix}x(3-x-2y)\\ y(2-x-y)\end{pmatrix}$$

## System 5

A mass-spring system is subject to damping and a nonlinear restoring force, modeled by the equation:

$$\ddot{z} + 2\beta\dot{z} + \alpha z + \gamma z^2 = 0$$

where:
* \\(z(t)\\) - displacement,
* \\(\beta = 0.5\\) - damping coefficient,
* \\(\alpha = 1\\) - linear stiffness coefficient,
* \\(\gamma = 1\\) - nonlinear coefficient.

<br>
<br>

# Role of a timestep in simulation
Let \\(x(t)\\) be the system state. If the system evolves according to \\(\dot{x} = f(x)\\) and the timestep is \\(\Delta t\\), the Forward Euler integration scheme is given by:
$$x_{i+1} = x_i + \Delta t f(x_i)$$
<br>

## Manual simulation step
For System 5, perform two simulation steps using the Forward Euler method with a timestep \\(\Delta t = 0.1\\), starting at the fixed points.

Will the system stay at the fixed points during the simulation? Why or why not? Does the result depend on the choice of timestep? How does the behaviour at the fixed points compare to that from any other initial state?

## Does this matter in real simulations?
When we increase the timestep, we speed up the simulation, but we also decrease its accuracy. With a sufficiently large timestep, unexpected things might happen. Objects may pass through each other, or collisions can launch objects in random directions at high velocities. Depending on the purpose of the simulation, it is important to adjust the timestep parameter.


In the `timestep.py` file, you are given a very simple system with a ball bouncing on a plane. The simulated bouncing is near-perfect, and thus the ball will bounce forever.

Run the program and observe the behaviour of the ball. Then increase the `TIMESTEP` variable and see how this affects the system. What is the threshold at which the physics seem to change?
