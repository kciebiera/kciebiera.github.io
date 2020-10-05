---
layout: post
title:  "Gripper pybullet"
date:   2020-10-01 08:00:00 +0200
---

How to build an oversimplified gripper in pybullet.

The python source code is available at : [https://github.com/kciebiera/grip_1](https://github.com/kciebiera/grip_1)

Program is able to grip some objects from the floor. Instead of using a real gripper I use two cuboids.

![Screenshot](/images/gripper.png)

## The world ##

The world consists of a huge [box](https://github.com/kciebiera/grip_1/blob/master/urdf/plane_black.urdf) (3m, 3m, 2m) located in (0, 0, -1) and a number of smaller [objects](https://github.com/kciebiera/grip_1/blob/master/urdf/object_demo.urdf).

```python
import pybullet as p
import random
serverMode = p.GUI # GUI/DIRECT
p.connect(serverMode)
p.setGravity(0, 0, -10)
planeID = p.loadURDF("./urdf/plane_black.urdf")

for i in range(20):
    object_id = p.loadURDF("./urdf/object_demo.urdf",
        [random.uniform(-0.4, 0.4),
         random.uniform(-0.4, 0.4),
         0.3],
        globalScaling=random.uniform(0.0015, 0.0025))

while True:
    p.stepSimulation()
```

We use `loadURDF` method. It allows us to specify URDF file (geometry, mass, colisions, ....), 3D position of loaded object and its scale. You should be able to see results of the simmulation, some objects are bouncing. If it is too fast you can add `sleep` within a simmulation.

## Gripper ##

Now lets take a closer look at [the gripper](https://github.com/kciebiera/grip_1/blob/master/urdf/fetch_gripper.urdf).

It consists of two `boxes` (I'm showing only the right one)

```xml
<joint name="r_gripper_finger_joint" type="prismatic">
    <origin rpy="0 0 0" xyz="0 0.005 0" />
    <parent link="gripper_link" />
    <child link="r_gripper_finger_link" />
    <axis xyz="0 1 0" />
  <limit effort="60" lower="0.0" upper="0.05" velocity="0.05" /><dynamics damping="100.0" /></joint>
  <link name="l_gripper_finger_link">
    <inertial>
      <origin rpy="0 0 0" xyz="-0.01 0 0" />
      <mass value="0.0798" />
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0" iyz="0" izz="0" />
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0" />
      <geometry>
        <box size="0.06 0.01 0.04"/>
      </geometry>
      <material name="a">
        <color rgba="0.356 0.361 0.376 1" />
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0" />
      <geometry>
        <box size="0.06 0.01 0.04"/>
      </geometry>
    </collision>
  </link>
```

Important parts are `<box size="0.06 0.01 0.04"/>`, so we have two (left, right) boxes of shape (6cm, 1cm, 4cm) and 
`<joint name="r_gripper_finger_joint" type="prismatic">`, so a box is joined to a `gripper_link` by an invisible [prismatic joint](https://en.wikipedia.org/wiki/Prismatic_joint) (it can move only in one direction).

The whole gripper is invisibly attached to "nothing" above the ground and has three prismatic joints (x, y, z sliders), one revolute joint (rotation) and two prismatic joints for left and right finger.

Let's test it:

```python
import pybullet as p
import numpy as np

robotUrdfPath = "./urdf/fetch_gripper.urdf"

p.connect(p.GUI)
p.setGravity(0, 0, -10)

planeID = p.loadURDF("./urdf/plane_black.urdf")

robotStartPos = [0, 0, 0.15]
robotStartOrn = p.getQuaternionFromEuler([0, 1.57, 0])
robotID = p.loadURDF(robotUrdfPath, robotStartPos, robotStartOrn)


p.addUserDebugParameter('x', -0.5, 0.5, 0)
p.addUserDebugParameter('y', -0.5, 0.5, 0)
p.addUserDebugParameter('z', -0.25, 1, 0.15)
p.addUserDebugParameter('roll', -3.14, 3.14, 0)
p.addUserDebugParameter("gripper_opening_length", 0, 0.03, 0.03)

while True:
    for i in range(4):
        p.setJointMotorControl2(
                robotID,
                i,
                p.POSITION_CONTROL,
                p.readUserDebugParameter(i))

    l = p.readUserDebugParameter(4)# "gripper_opening_length"
    p.setJointMotorControl2(robotID, 4, p.POSITION_CONTROL, targetPosition=l)
    p.setJointMotorControl2(robotID, 5, p.POSITION_CONTROL, targetPosition=l)
    p.stepSimulation()
```

![Screenshot](/images/gripper-2.png)

We can now play with our gripper. Try it.

## Camera ##

The last missing piece is a camera for taking photos.

```python
width = 256  # px
height = 256 # px
fov = 90     # Field-of-view in degrees
aspect = 1.0 # width / height
near = 0.2   # minimum distance of rendering
far = 2      # maximum distance of rendering
view_matrix = p.computeViewMatrix([0, 0, 0.5], [0, 0, 0], [0, 1, 0]) #position, direction, top vector
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
width, height, rgb_img, depth_img, seg_img = p.getCameraImage(
    width, height, view_matrix, projection_matrix,
    renderer=p.ER_BULLET_HARDWARE_OPENGL)
```

## Logic ##

Now, as we have all components needed for simmulation (world, objects, gripper, camera) we need some logic for gripping. In this post we use:

- actual positions of object centers before grabbing `x, y, _ = p.getBasePositionAndOrientation(object_id)[0]`,
- we check in function `grab_at` if an objects height has increased by at least 10cm,
- we remove an object after successfull pickup `p.removeBody(object_id)`.


<video width="600" controls="controls">
  <source src="/images/grip.mp4">
</video>