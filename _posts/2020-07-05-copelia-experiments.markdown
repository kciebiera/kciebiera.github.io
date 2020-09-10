---
layout: post
title:  "UR5 Inverse Kinematics in CoppeliaSim"
date:   2020-07-05 12:00:00 +0200
---


How to pickup an object using a robotic arm simmulator in a CoppeliaSim (formerly V-Rep).

## Running Coppelia ##

### Installation ###

We need to download educational version from (https://www.coppeliarobotics.com/downloads) and unpack it.

### Running ###

Just start `./coppeliaSim.sh` file located in main directrory of Coppelia folder.

## Assemble a robot ##

Before performing any experiments we need to assemble  a robot first. We will use UR5 model together with Baxter gripper. So we run CoppeliaSim and the we should see following empty scene.

![CoppeliaSim](/images/make_robot_1.png)

From the model browser on the left (it is hidden/shown using small humanoid robot button) we can drag parts and drop them to the scene. We drag UR5 from robots->mobile node of the model browser and drop it on the scene.

![Add robot](/images/make_robot_2.png)

In order to join robot and the gripper we need to see red dot at the tip of the robot. Therefore we need to rotate the scene (second button from the left at the top of a window).

![Rotate](/images/make_robot_3.png)

Next we need to add a gripper. We drag Baxter gripper from the model browser and drop in onto the scene.

![Add gripper](/images/make_robot_4.png)

In the last step we attach gripper to the robot. It's tricky, we need to:

1. Select (click) the gripper in the scene.
2. Select (ctrl-click) the red dot on the tip of the robot.
3. Join (9th button from the left) selected elements.

![Join robot and gripper](/images/make_robot_5.png)

## Prepare inverse kinematics ##

Inverse Kinematics (IK) in CoppeliaSim can work two different modes:

- Pseudo inverse - I assume it is based on pseudo inverse of Jakobian, so it should work always if the target position is achievable.
- DLS - Damped Least Squares, it should be more stable especially when target position is not achievable.

### Set joints properties ###

Standard way of handling robot joints is Torque/Force mode. We want joints to be controlled by IK, so we need to set then into IK mode. Note that we need to enable _hybrid operation_ mode
and it needs to be enabled for all six joints `(UR5_joint[1..6])`

![Join robot and gripper](/images/joint_mode.png)

### Remove original script for UR5 ###

We need to remove original "dancing script" for UR5

1. Select `UR5` in scene browser
2. Select `Edit->Remove->Associated child script` from menu

### Link tip to a target ###

We now will add `tip` to the robot and `target` to the scene. IK will calculate how robot joints will be moved in order for `tip` to reach the `target`.

1. Add Dummy (right click) to the scene and rename it as a `tip`
2. Drag and drop it as a child of `UR5_link7` in scene browser
3. Set `tip` position as `{0,0,0}` related to its parent.
![Tip position](/images/tip_position.png)
4. Add another Dummy call it `target` and move it to achievable point.
5. Select both (ctrl-click) `tip` and `target` and join them right click `Edit->Link selected dummies->IK, tip-target`

### Add IK group ###

This is the last step of setting up IK. We want to say that `tip` should follow the `target`.

1. Select `Calculation module parameters` on the left third button from top.
2. Select `Kinematics`
![Kinematics window](/images/kinematics_window.png)
3. Click `Add new IK group`
4. Click `Edit IK elements` (new window should be shown)
5. Select `tip` and click `Add new IK element with a tip`
6. Close windows

The result should be similar to:

<video width="600" controls="controls">
  <source src="/images/robot_movement.webm">
</video>
