# Unitree Go1 robot dog

This quadruped robot comes with out-of-the-box behaviours but can also be run on custom code. 

## 1. How to work with the Unitree Go1 robot dog
The Unitree Go1 robot dog (Go1 or dog for short) offers you a number of ways to use it. Therefore, you have to make some choices before you get started: Running mode and programming language.

The dog has two running modes: High level and low level. If you choose to use high level control, you can use the preprogrammed behaviour such as walking without having to worry about controlling the individual motors. This can be useful if your proposed algorithm is not dependent on the motor values, e.g. line following. If instead, you're trying to teach the robot something like a different walking style, then choose low level which gives to access to the individual motor values.

A second choice concerns the programming language you intend to use. Unitree offers a python wrapper but it is also possible to use ROS or ROS2. We only provide guidance for the use of the python wrapper here, however, so that is the recommended choice.

If you just want to get a feel for the dog, we suggest following the instructions for high level setup and joystick control.

## 2. Unitree Go1 robot dog setup

Before getting started with the code, you first need to know how to set up the dog to work with your chosen running mode.

### Preparing the Unitree Go1 robot dog (both modes)
High level control is the default mode of the robot dog but these steps also need to be completed before getting into low level control.

Before turning on the robot, make sure it is placed on a level surface (preferrably the ground) and laying flat with its belly to the ground. Check that the body is not tilted and that the robot's calves are fully retracted. Also make sure that the robot's thighs and calves are not under any pressure from the body, otherwise the robot may fail to boot.

Once the robot is in its initial position, press the power button once followed by a second press for 3s to turn on the battery. When the battery is turned on, the indicator light is green and remains on showing the current battery level. Wait until the robot completes the power-on self-test. If this test is successful, the dog will stand up and the boot is successful. In this case, you are ready to continue to the control section.

If the robot does not stand up, the robot failed the self-test. Most likely either the battery is empty or the dog is not positioned correctly. Reposition the body and check the battery level.

### Setting up the Unitree Go1 robot dog in low level mode
To get into low level control, you need to prepare the robot dog carefully or you risk damaging the dog.

If you have not yet done so, follow the steps above to turn the robot on and get it to stand up. After doing so, switch the remote control on by giving the power button a short press followed by a 3s press. Press L2 + A twice to lower the robot dog. Then, press L2 + B to release the joints. Finally, press L1 + L2 + Start to switch into low level mode. 

**Important**: Before sending any commands to the dog, you need to hang it up, e.g. suspend it from the ceiling. It needs to hang without touching the floor or anything else.


## 3. Unitree Go1 robot dog control
There are multiple ways of controlling the dog. We introduce two here: remote control and via the python wrapper.

### Remote control via joystick
Press the power button at the base of the remote once and then press it a second time for 3s. This both powers the remote on and later off.

The left joystick controls the movement direction of the dog (forward, backward, left, right), the right joystick can be used to turn the dog.

### Control via python wrapper


## Powering off
