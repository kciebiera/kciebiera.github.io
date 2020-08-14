---
layout: post
title:  "How to move UR5 with gripper in CoppeliaSim"
date:   2020-07-05 12:00:00 +0200
categories: vue
---

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

