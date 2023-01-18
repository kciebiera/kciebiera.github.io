---
title: Interbotix WidowX-250 Environement Installation
date:   2023-01-18 15:00:00 +0200
---

In this post, we will install the environment for the WidowX-250 robot. The environment is composed of the following components:

- Ubuntu 18.04
- ROS Melodic
- Interbotix SDK
- Exaple python2 code (yes, python2, sorry)

## Ubuntu 18.04

I am trying to do it as simple as possible, therefore I assume you have access to any modern Ubuntu version. The
method probably works on other Linux distributions as well, but I have not tested it, also on Windows and on Intel Macs. It does not work on ARM Macs (yet).

We start by using multipass (https://multipass.run/) to create a virtual machine with Ubuntu 18.04. Multipass is a tool to create virtual machines on your computer. It is very easy to use and it is free. You can install it on your computer by following the instructions on the website. Once installed, you can create a virtual machine with the following command:

```bash
multipass launch -n viper-lab -d 20G -m 4G -c 2
```

From this moment on, you can access the virtual machine by using nice widget in the top bar of your computer (look at image below):

![Multipass](/assets/images/wx250/multipass.jpg)

You can also use the command line to access the virtual machine. The following command will open a terminal in the virtual machine:

```bash
multipass shell viper-lab
```

## ROS Melodic

Now let's NOT install ROS Melodic. It will be automatically installed when we install the Interbotix SDK. We will install the Interbotix SDK first.

## Interbotix SDK

The Interbotix SDK is a collection of tools to control the WidowX-250 robot. It is available on GitHub. Open multipass shell and install the SDK with the following commands:

```bash
curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.shc
chmod +x xsarm_amd64_install.sh 
./xsarm_amd64_install.sh -d melodic
```

You should answer `no` to questions about installing Camera drivers and matlab, and at the end you should answer `yes` to the question about installing the Interbotix SDK.

## X Server

The Interbotix SDK uses the X Server to display the robot in RViz. We need to install the X Server on the virtual machine. I recomend using X11 forwarding as described in https://multipass.run/docs/set-up-a-graphical-interface#heading--x11-on-linux.

## Example code

Now lets test it. First let's clone the example code:

```bash
ubuntu@viper-lab:~$ git clone https://github.com/Interbotix/interbotix_ros_manipulators.git
```

Now in two separate ssh sessions, run the following commands:

```bash
roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250 use_sim:=true
```

it should display RViz with the robot in it.

```bash
ubuntu@viper-lab:~$ cd interbotix_ros_manipulators/interbotix_ros_xsarms/examples/python_demos/
ubuntu@viper-lab: $ python bartender.py
```

RViz should display the robot moving. It is trying to move an invisible bottle of beer like in the image below:

![Bartender](/assets/images/wx250/bartender.png)

## Conclusion

We have installed the environment for the WidowX-250 robot. It can be used to control the robot in simulation. The whole code controlling the robot follows:

```python
from interbotix_xs_modules.arm import InterbotixManipulatorXS
import numpy as np

# This script makes the end-effector perform pick, pour, and place tasks
#
# To get started, open a terminal and type 'roslaunch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250'
# Then change to this directory and type 'python bartender.py  # python3 bartender.py if using ROS Noetic'

def main():
    bot = InterbotixManipulatorXS("wx250", "arm", "gripper")
    bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    bot.arm.set_single_joint_position("waist", np.pi/2.0)
    bot.gripper.open()
    bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16)
    bot.gripper.close()
    bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)
    bot.arm.set_single_joint_position("waist", -np.pi/2.0)
    bot.arm.set_ee_cartesian_trajectory(pitch=1.5)
    bot.arm.set_ee_cartesian_trajectory(pitch=-1.5)
    bot.arm.set_single_joint_position("waist", np.pi/2.0)
    bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16)
    bot.gripper.open()
    bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
```
