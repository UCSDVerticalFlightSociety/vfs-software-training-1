# Vertical Flight Society Training Module
Hello new Software Members! This is meant to be an introductory training module of the Robotics Operating System 2 (ROS). Understanding ROS will significantly improve your ability to write effective and adaptable code for this team since it's the inspiration for many other robotics software frameworks.

This module focuses on the core ROS concept of **Publishers and subscribers** and provides you with the opportunity to apply your learnings from documentation to actually implementing your knowledge.

## Overview
You will learn:
* How to listen to topics
* Process data you have received
* Publish to topics
([ROS docs for Publishers and Subscribers](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html))  

Your goal is to create a node that acts as a simple drone navigator. You will apply your model to:
1. Subscribe to drone's current position `/currentpos` and target position `/targetpos` topics.
    * Both topics publish a position vector every half a second.
    * The elements of these vectors are always rounded to the nearest hundredth.
2. Publish calculated velocity vector to drone's velocity `/drone_vel`
    * Your maximum X and Y velocities are capped at 20 units per update.
3. Once you arrive at your target, you must remain at the target position for 5 updates in order to complete the task.

  _**CATCH:** Just like in real life, the wind blows the drone a couple units in a random direction on every update. So consider that when making your solution (hint: its not very complicated)._


Your final solution should look something like this (drone is represented by `H` and target is represented by `0`:

[Screencast from 11-18-2025 12:20:06 PM.webm](https://github.com/user-attachments/assets/5dc288af-68da-4b79-a13e-0c320614963f)

## ROS2 Installation
**You must have ROS2 Humble installed before proceeding.**
Refer to the [ROS2 Humble Documentation](https://docs.ros.org/en/humble/index.html).

***!! IF YOU ARE A MacOS USER !!***
If you are using a MacBook, we require you to install an <ins>Ubuntu</ins> Virtual Machine (VM) using <ins>UTM</ins> for a consistent development environment. Please follow this [video tutorial](youtube.com/watch?si=pxCuEIL9hvmRuVzZ&v=MVLbb1aMk24&feature=youtu.be).

Steps:
1. Create and build your own workspace
    * Follow the official [ROS 2 documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) to set up your environment. Stop after the colcon build step and ignore the Turtlesim tutorial.
2. Run the module
    * After building, you *must* source your overlay in a **NEW TERMINAL** before running the module.

## Getting started

                                        ** Read the section in its entirety before getting started! **
What you'll do:
* Source ROS2 to make its commands available in your terminal.
* Install packages
  * A package is an organizational unit consisting of a collection of nodes, interfaces, etc.
  * Refer to [ROS2 package tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html).  

Steps:
1. Source ROS2 (Underlay)

Download [colcon](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)(optional) and rosdep.
These are the two package tools ROS2 uses in package management.
To install, run:
```
sudo apt install python3-colcon-common-extensions
apt-get install python3-rosdep
#this only needs to be done once
sudo rosdep init
rosdep update
```

To actually run the package, your system needs to `source` ros commands. This will create a [workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html).
Run:
```
source /opt/ros/humble/setup.bash
```
* `/` in `/opt` means we go to the root of our system, then search the `opt` folder. We keep going down the file tree until we find `setup.bash`, then run that.
* **This is required in every new terminal before running ROS commands.**

2. Clone, Install, Build (Overlays)

Download the code from this GitHub repository and `cd` into it.
Run:
```
git clone https://github.com/UCSDVerticalFlightSociety/vfs-software-training-1
```
cd into the folder, install any necessary dependencies, then build your package.
You will have a setup file to source your overlay workspace.
See [ROS2 docs for creating a workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).

Your total set of commands will look like this:
```
source /opt/ros/humble/setup.bash
cd vfs-software-training-1
rosdep install -i --from-path src --rosdistro humble -y
colcon build
#IN A NEW TERMINAL
cd vfs-software-training-1
source install/setup.bash
```
Run the package:
```
ros2 run navigation_py navigation_py
```
Congratuations! You can now start building your solution.
Whenever a file change is made, you simply need to run colcon build (which will rebuild your package), and run it again.

## Understanding your environment
There is only one [node](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)(optional) running.
To check, run:
```
~/Code/VFS/vfs-software-training-1$ ros2 node list
/drone_node
```

However, there are multiple [topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html) (optional): 
```
~/Code/VFS/vfs-software-training-1$ ros2 topic list
/currentpos
/drone_vel
/parameter_events
/rosout
/targetpos
```
These topics are publishing messages, defined by [interfaces](https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html).
Each object publishes a type of message:
```
~/Code/VFS/vfs-software-training-1$ ros2 topic list -t
/currentpos [std_msgs/msg/Float64MultiArray]
/drone_vel [std_msgs/msg/Float64MultiArray]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/targetpos [std_msgs/msg/Float64MultiArray]
```
To understand the structure of the interfaces you will working with (Float64MultiArray), run:
```
~/Code/VFS/vfs-software-training-1$ ros2 interface show std_msgs/msg/Float64MultiArray
```
For more information, read [here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html) (optional).  


The main topic we care about is the `/currentpos` topic.
```
~/Code/VFS/vfs-software-training-1$ ros2 topic echo /currentpos
```
You will be listening to, processing, and making your calculation with this data and for the `/targetpos` topic.  

* Create the `navigator` node in the `your_solution` package.
  * This should publish to `/drone_vel`, with a message type of `ArrayMsg = std_msgs::msg::Float64MultiArray`.
  * x-coord is the 0th entry
  * y-coordinate is the 1st entry.

Your final structure will look like this:
<img width="1404" height="548" alt="image" src="https://github.com/user-attachments/assets/4879122a-4294-4d40-b76d-f998fceacce9" />

## What you need to do
* Create the [node](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html).
* Subscribe to both `/targetpos` and `/currentpos` topics
* Publish the desired velocity.
Your final solution will look like this:

[Screencast from 11-18-2025 12:20:06 PM.webm](https://github.com/user-attachments/assets/5dc288af-68da-4b79-a13e-0c320614963f)

### When complete
* Take a screenshot of the terminal displaying your success.
* Push your code and the image to your github repository.
<img width="722" height="75" alt="image" src="https://github.com/user-attachments/assets/98936293-c875-4b7d-914f-c67e13e7a32e" />

Talk to your software lead and have them review your code. They will give you the next steps from there.
