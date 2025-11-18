# Vertical Flight Society Training Module
Hello new Software Members! This is meant to be an introductory training module to ROS. ROS itself is an integral skill to any sort of robotics in the future, and is the inspiration for many other robotics software frameworks. PX4, for example, uses the same concepts in their design and so like learning the programming language C, learning ROS will make you adaptable and effective at writing code for this team.
This module will introduce you to a core idea of ROS: Publishers and subscribers. It isn't enough to walk through the documentation on the Humble docs. Only once you implement your knowledge will you understand what you are actually doing. This module is meant to be that opportunity to apply your knowledge.
## Overview
In this module, you will learn how to listen to topics, process the data you have received, and publish to topics ([ROS docs for Publishers and Subscribers](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html))  

You will have two topics provided to you: `/currentpos` and `/targetpos`, each which publish a position vector every half a second. The elements of these vectors are always rounded to the nearest hundredth.  
It is your job to calculate and publish velocity vector the drone of the drone based on these given positions. Your maximum X and Y velocities are capped at 20 units per update. Finally, once you arrive at your target, you must remain at the target position for 5 updates in order to complete the task.
There is a catch though: the wind blows the drone a couple units in a random direction on every update, just like in real life. So consider that when making your solution (hint: its not very complicated).  

Your final solution should look something like this (drone is represented by `H` and target is represented by `0`:

[Screencast from 11-18-2025 12:20:06 PM.webm](https://github.com/user-attachments/assets/5dc288af-68da-4b79-a13e-0c320614963f)

## Getting started
Read the section in its entirety before getting started!  
Let's get set up with our packages. A package is essentially a organizational unit consisting of a collection of nodes, interfaces, and launch files, among other things. You can read more in the [ROS2 package tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html).  

But first, we need to download [colcon](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)(optional) and rosdep, the two package tools ROS2 uses in package management. These can be installed by running
```
sudo apt install python3-colcon-common-extensions
apt-get instal python3-rosdep
#this only needs to be done once
sudo rosdep init
rosdep update
```

To actually run this package, your system needs to `source` ros commands. This will create a [workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html). Do this by running the following command: 
```
source /opt/ros/humble/setup.bash
```
The `/` in `/opt` means we are going to the root of our system, and then searching the `opt` folder. We keep going down the file tree until we find `setup.bash` and then we run that.
You have to do this any time you want to use ros commands. In fact, it's such a good practice that some people put it in their configuration files so that this command is automatically run when they open their terminal!  

Now that you know how to source the underlay, its time to create our overlays. Download the code from this github repository and `cd` into it. Run
```
git clone https://github.com/UCSDVerticalFlightSociety/vfs-software-training-1
```
Once you cd into the folder, you need to install any necessary dependencies, and then build your package. After that, you will have a setup file to source your overlay workspace. See [the ros docs for creating a workspace for reference](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
Your total set of commands will look like this.
```
source /opt/ros/humble/setup.bash
cd vfs-software-training-1
rosdep install -i --from-path src --rosdistro humble -y
colcon build
#IN A NEW TERMINAL
cd vfs-software-training-1
source install/setup.bash
```
Finally, you can run the package.
```
ros2 run navigation_py navigation_y
```
Congratuations! You can now start building your solution. Whenever a file change is made, you simply need to run colcon build (which will rebuild your package), and run it again.
## Understanding your environment
Currently, there is only one [node](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)(optional) running. You can check this by running:
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
These topics are publishing messages, which are defined by [interfaces](https://docs.ros.org/en/humble/Concepts/Basic/About-Interfaces.html). To see the type of message that each object is publishing, you can run:
```
~/Code/VFS/vfs-software-training-1$ ros2 topic list -t
/currentpos [std_msgs/msg/Float64MultiArray]
/drone_vel [std_msgs/msg/Float64MultiArray]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/targetpos [std_msgs/msg/Float64MultiArray]
```
If you want to understand the structure of the interfaces you are going to be working with (which is Float64MultiArray), you can check that by visualizing the interface type:
```
~/Code/VFS/vfs-software-training-1$ ros2 interface show std_msgs/msg/Float64MultiArray
```
If you are curious, read more about that [here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html) (optional).  
The main topic we care about is the /currentpos topic.
```
~/Code/VFS/vfs-software-training-1$ ros2 topic echo /currentpos
```
This is the data you will be listening to, processing, and making your calculation with. Same goes for the `/targetpos` topic.  

To do this, you are going to create the `navigator` node in the `your_solution` package. It will publish to `/drone_vel`, with a message type of `ArrayMsg = std_msgs::msg::Float64MultiArray`. The x-coordinate is the 0th entry, and the y-coordinate is the 1st entry.
Your final structure will look like this:
<img width="1404" height="548" alt="image" src="https://github.com/user-attachments/assets/4879122a-4294-4d40-b76d-f998fceacce9" />

## What you need to do
Create the [node](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html). Have it subscribe to both the `/targetpos` and `/currentpos` topics. Then have it publish the desired velocity.
Your final solution will look like this:
[Screencast from 11-18-2025 12:20:06 PM.webm](https://github.com/user-attachments/assets/5dc288af-68da-4b79-a13e-0c320614963f)

### When complete
Talk to your software lead and have them review your code. They will give you the next steps from there.
