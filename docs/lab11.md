<script type="text/javascript"

  src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.3/MathJax.js?config=TeX-AMS-MML_HTMLorMML">

</script>
# LQR control



$$\ddot{\theta} = \frac{(M+m)g\sin\theta - \cos\theta \left[ F + ml\dot{\theta}^2 \sin\theta \right]}{\left( \frac{4}{3} \right)(M+m) - ml\cos^2\theta}$$

$$\ddot{x} = \frac{ \left\{ F + ml \left[ \dot{\theta}^2 \sin\theta - \ddot{\theta}\cos\theta \right] \right\} }{M+m}$$


In this class we will implement LQR control for a cartpole system. We will use the [control](https://python-control.readthedocs.io/) library to design the controller and simulate the system.

The lab consists of three parts:

1. We will use the `control` library for a simple linear system. Just to get a feel for the library.
2. We will use the `control` library for a cartpole system. We will use the linearized dynamics of the cartpole system. The system will be simulated using  https://github.com/microsoft/cartpole-py/blob/main/cartpole.py which is a Python implementation of the cartpole system from the Microsoft.
3. We will use the `control` library for a cartpole system simmulated using MuJoCo.

In the second step, you need to linearize the cartpole system at a fixed point using the basic simulation source code you have. Then, employ this linearization in the third step.

## Part 1: Simple linear system

We have a simple linear System

```python
import control as ctrl
import numpy as np

# Define system matrices
A = np.array([[1, 2], [3, 4]])
B = np.array([[5], [6]])

class System:
    def __init__(self):
        self.x = np.array([[1], [2]])  # Initial state

    def measure_state(self):
        return self.x

    def apply_control(self, u):
        dt = 0.01
        self.x = self.x + np.dot(A, self.x) * dt + np.dot(B, u) * dt

    def print_state(self):
        print(f"{self.x[0].item():.2f}, {self.x[1].item():.2f}")
```

The system is simulated using the following code:

```python
system = System()

for _ in range(10):
    x = system.measure_state()
    # Compute control signal
    signal = random.uniform(-1, 1)

    system.apply_control(signal)
    system.print_state()
```

If you run the code, you should see the system moves randomly. We want to design a controller that stabilizes the system to the origin `[0,0]`. We will use the `control` library to design the controller.

```python
# Define cost matrices
Q = np.array([[1, 0], [0, 1]])
R = np.array([[1]])

# Compute LQR controller gain
K, S, E = ctrl.lqr(A, B, Q, R)

# K is your controller gain
print("Controller gain K:", K)
```

Now, we can use the controller gain `K` to stabilize the system. We will use the following code to simulate the system:

```python
for _ in range(1000):
    # Measure or estimate the current state
    x = system.measure_state()

    # Calculate the control input
    u = -np.dot(K, x)

    # Apply the control input to the system
    system.apply_control(u)

    # Wait for the system to react before next iteration
    # (This could be a time delay in a real-time system)
    time.sleep(0.001)

    # Print the current state
    system.print_state()
```

Try to run the code. You should see that the system is stabilized to the origin.

Check what happens if you change the cost matrices `Q` and `R`. What happens if you increase the cost of the control signal `R`? What happens if you increase the cost of the state `Q`? Make a plot of the state trajectory for different values of `Q` and `R`.

## Part 2: Cartpole system - using specific simulation code

In this part, we will use the cartpole system from Microsoft. The code is available at https://github.com/microsoft/cartpole-py/blob/main/cartpole.py, but we have included it here for convenience (with some minor modifications).

```python
"""
Classic cart-pole system implemented by Rich Sutton et al.
Derived from http://incompleteideas.net/sutton/book/code/pole.c
"""
__copyright__ = "Copyright 2020, Microsoft Corp."


import math
import argparse
import random
from typing import Any
import cv2
import numpy as np
import control

# Constants
GRAVITY = 9.8  # a classic...
CART_MASS = 0.31  # kg
POLE_MASS = 0.055  # kg
TOTAL_MASS = CART_MASS + POLE_MASS
POLE_HALF_LENGTH = 0.4 / 2  # half the pole's length in m
POLE_MASS_LENGTH = POLE_MASS * POLE_HALF_LENGTH
FORCE_MAG = 1.0
STEP_DURATION = 0.02  # seconds between state updates (20ms)
TRACK_WIDTH = 1.0  # m
FORCE_NOISE = 0.02  # % of FORCE_MAG


# Model parameters
class CartPoleModel:
    def __init__(self, initial_cart_position: float = 0, initial_pole_angle: float = 0):
        # cart position (m)
        self._cart_position = initial_cart_position

        # cart velocity (m/s)
        self._cart_velocity = 0

        # cart angle (rad)
        self._pole_angle = initial_pole_angle

        # pole angular velocity (rad/s)
        self._pole_angular_velocity = 0

        # pole position (m)
        self._pole_center_position = 0

        # pole velocity (m/s)
        self._pole_center_velocity = 0

    def step(self, command: float):
        # We are expecting the input command to be -1 or 1,
        # but we'll support a continuous action space.
        # Add a small amount of random noise to the force so
        # the policy can't succeed by simply applying zero
        # force each time.
        force = FORCE_MAG * command + random.uniform(-FORCE_NOISE, FORCE_NOISE)

        cosTheta = math.cos(self._pole_angle)
        sinTheta = math.sin(self._pole_angle)

        temp = (
            force + POLE_MASS_LENGTH * self._pole_angular_velocity**2 * sinTheta
        ) / TOTAL_MASS
        angularAccel = (GRAVITY * sinTheta - cosTheta * temp) / (
            POLE_HALF_LENGTH * (4.0 / 3.0 - (POLE_MASS * cosTheta**2) / TOTAL_MASS)
        )
        linearAccel = temp - (POLE_MASS_LENGTH * angularAccel * cosTheta) / TOTAL_MASS

        self._cart_position = self._cart_position + STEP_DURATION * self._cart_velocity
        self._cart_velocity = self._cart_velocity + STEP_DURATION * linearAccel

        self._pole_angle = (
            self._pole_angle + STEP_DURATION * self._pole_angular_velocity
        )
        self._pole_angular_velocity = (
            self._pole_angular_velocity + STEP_DURATION * angularAccel
        )

        # Use the pole center, not the cart center, for tracking
        # pole center velocity.
        self._pole_center_position = (
            self._cart_position + math.sin(self._pole_angle) * POLE_HALF_LENGTH
        )
        self._pole_center_velocity = (
            self._cart_velocity
            + math.sin(self._pole_angular_velocity) * POLE_HALF_LENGTH
        )

    def halted(self):
        # If the pole has fallen past 45 degrees, there's no use in continuing.
        return abs(self._pole_angle) >= math.pi / 4

    def state(self):
        return {
            "cart_position": self._cart_position,
            "cart_velocity": self._cart_velocity,
            "pole_angle": self._pole_angle,
            "pole_angular_velocity": self._pole_angular_velocity,
            "pole_center_position": self._pole_center_position,
            "pole_center_velocity": self._pole_center_velocity,
        }


def visualize(state):
    width = 500
    image = np.zeros((400, width, 3), np.uint8)
    cart_x = int(state["cart_position"] * 50) + width // 2
    angle = state["pole_angle"]  # in radians
    print(cart_x, angle)
    pole_x = int(math.sin(angle) * 100) + cart_x
    cv2.line(image, (cart_x, 350), (pole_x, 50), (255, 255, 255), 3)
    cv2.circle(image, (cart_x, 350), 10, (255, 255, 255), -1)
    cv2.imshow("pole", image)
    cv2.waitKey(1)


def print_state(state, name):
    print(
        name,
        f'{state["cart_position"]:.4f}, {state["cart_velocity"]:.4f}, {state["pole_angle"]:.4f}, {state["pole_angular_velocity"]:.4f}',
    )



def no_force():
    pole = CartPoleModel()
    state = pole.state()
    for i in range(10000):
        if pole.halted():
            print("halted")
            break
        pole.step(0)
        state = pole.state()
        visualize(state)
        print_state(state, "current")

no_force()
```

When you run the code, you should see the cartpole system in action. Since there is no controller, the system will fall down (the pole angle will be greater than 45 degrees) as seen on a video below.


<video width="512" height="208" controls>
  <source src="simple-free.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

Check what happens if you change the initial cart position and pole angle. What happens if you remove the noise from the force? What happens if you change the mass of the cart or the pole? What happens if you change the length of the pole? What happens if you change the gravity constant (ok, it's a joke)?

Your task is to design an LQR controller that stabilizes the cartpole system to the origin.The linearization should be done at a fixed point.
You should write a function that returns the matrices `A` and `B` of the linearized system.  You should know how to linearize the system from the previous classes. 

```python
def linearize(): # you can add arguments if you want
    #TODO
    
    A = np.array(
    #TODO
    )
    B = np.array(
    #TODO
    )
    return A, B


def controlled():
    pole = CartPoleModel(initial_cart_position=-3)
    state = pole.state()
    A, B = linearize() #maybe add arguments
    Q = # TODO
    R = # TODO
    K = control.lqr(A, B, Q, R)[0]
    print("K", K)
    for i in range(10000):
        if pole.halted():
            print("halted")
            break
        force = -(
            K
            @ np.array(
                [
                    state["cart_position"],
                    state["cart_velocity"],
                    state["pole_angle"],
                    state["pole_angular_velocity"],
                ]
            )
        )[0]
        if force > 1:
            force = 1
        if force < -1:
            force = -1
        print("force", force)
        pole.step(force)
        state = pole.state()
        visualize(state)
        print_state(state, "current")

controlled()
```

After filling in the missing parts (marked by comments and TODOs), you should see the cartpole system stabilized to the origin as seen on a video below.


<video width="512" height="208" controls>
  <source src="simple-controlled.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

Please note, that system starts in -3 position, so it is not stabilized in the center. Also note, that we limit the force to be between -1 and 1.

## Part 3: Cartpole system - using MuJoCo

In this part, we will use the cartpole system from MuJoCo. Cartpole XML is a modified version of the cartpole.xml from MuJoCo.

```xml
<mujoco model='test_cartpole'>
  <compiler inertiafromgeom='true' coordinate='local'/>

  <option timestep='0.01'/>

  <worldbody>
    <camera name='fixed' pos='0 -2.5 0' quat='0.707 0.707 0 0'/>
    <geom name='floor' pos='0 0 -1' size='4 4 4' type='plane' />
    <body name='cart' pos='0 0 0'>
      <joint name='slider' type='slide' limited='true' pos='0 0 0' axis='1 0 0' range='-5 5' />
      <geom name='cart' type='box' pos='0 0 0' size='0.2 0.1 0.05' rgba='0.7 0.7 0 1'/>
      <body name='pole' pos='0 0 0'>
        <camera name='pole' pos='0 -2.5 0' quat='0.707 0.707 0 0' />
        <joint name='hinge' type='hinge' pos='0 0 0' axis='0 1 0'/>
        <geom name='cpole' type='capsule' fromto='0 0 0 0 0 0.5' size='0.06 0.001' rgba='0 0.7 0.7 1'/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <motor name='slide' joint='slider' gear='50' ctrllimited='true' ctrlrange='-1 1' />
  </actuator>

</mujoco>
```

Using the following stub code, you should be able to stabilize the cartpole system using MuJoCo.

```python
def sim_reset():
    global model, data, viewer_window
    if "viewer_window" in globals():
        viewer_window.close()
    model = mujoco.MjModel.from_xml_path("mujoco_cartpole.xml")
    renderer = mujoco.Renderer(model, height=480, width=640)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    if VIEW:
        viewer_window = viewer.launch_passive(model, data)


def sim_step(forward):
    data.actuator("slide").ctrl = forward
    step_size = 0.01
    step_start = time.time()
    mujoco.mj_step(model, data)

    if VIEW:
        viewer_window.sync()
        time_until_next_step = step_size - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


def run():
    for _ in range(50):
        sim_step(0.03)
    mass_cart = 8
    mass_pole = 6.5
    length_pole = 0.6

    A, B = linearize()# TODO
    Q = # TODO
    R = # TODO
    K = control.lqr(A, B, Q, R)[0]
    print("K", K)
    for _ in range(1000):
        angle = data.joint("hinge").qpos[0]
        angle_vel = data.joint("hinge").qvel[0]
        xpos = data.body("cart").xpos[0]
        xvel = data.body("cart").cvel[3]
        state = np.array([xpos, xvel, angle, angle_vel])
        sim_step(-(K @ state)[0])


sim_reset()
run()
print(model.body_mass)
```

You should see the cartpole system stabilized to the origin as seen on a video below. Note, that
at the beginning the system is not stabilized in the center because it takes 50 steps with force 0.03 as
in provided code.

<video width="500" height="400" controls>
  <source src="mujoco-controlled.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
