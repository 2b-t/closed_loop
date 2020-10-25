# Closed loops in ROS and Gazebo with SDF-code injection - Four-bar linkage example

*Author: [Tobit Flatscher](https://github.com/2b-t) (April 2020)

## Overview
This is a workaround on how to **include parallel robots into Robot Operating System ROS (URDF) and GazeboSim (SDF) by using Xacro** without having to retype the geometry manually twice for the URDF and the SDF-files.

### The issue
The Unified Robot Description Format (UDRF) only supports serial chains (meaning a parent may have multiple child nodes but a child only a single parent node) and therefore can't describe parallel robots due to their graph structure with closed loops (two parents for at least one link). Simulation with Gazebo on the other hand supports parallel geometries such as the Simulation Description Format (SDF). As a consequence both files are often written manually, making it error prone in case the geometry changes.
A [previous workaround](https://github.com/wojiaojiao/pegasus_gazebo_plugins) (that is anyways more complicated) did not work for me, so I had to come up with my own solution. 
In this workaround **closed loops are broken up into serial chains** and the **missing joints** are supplied to only Gazebo with corresponding instructions in **SDF**. Furthermore it introduces **XML macros with Xacro** to reduce code redundancy. In the process a single UDRF that can be saved on the ROS parameter server is generated that still contains the SDF to **automatically recover the full parallel geometry in GazeboSim**.

### The files
The files simulate a simple four-bar linkage similar to the example in SDF by [The Construct](https://youtu.be/hglRGiNHRno) that collapses to the side under gravity.
- `model/closed_loop.xacro` is the main file that *loads the components located in the other two Xacro files*
- `model/parameters.xacro` contains the *geometry parameters* such as height, depth, width, mass and macros for the inertial matrix
- `model/element.xacro` contains macros for a single element and *creates the entire geometry*
- `launch/closed_loop.launch` is the *launch-file* for the ROS parameter server and loads the geometry into Gazebo

## Launch my solution
Copy this folder to the source folder of your catkin workspace (e.g. `~/catkin_ws/src`) or directly **clone this repository** by typing
```
$ cd ~/catkin_ws/src 
$ git clone https://github.com/2b-t/closed_loop.git 
```
**Rebuild your workspace** with
```
$ cd ~/catkin_ws
$ catkin build
```
**Launch Gazebo_ROS** by typing
```
$ roslaunch gazebo_ros empty_world.launch
```
and my corresponding **launch file** that launches the ROS parameter server and loads the UDRF-model into GazeboSim with
```
$ roslaunch closed_loop closed_loop.launch
```
This should bring up a simple four-bar linkage in GazeboSim that collapses to the side.

## Adapt it to your robot
In order to modify the above code for your robot create a Xacro-file for the **parallel robot with a single leg only** at first. Your robot is likely under-determined and the end-effector would simply fall to the floor if you would simulate this geometry.
Then **create the URDF-file** from the Xacro macros with this simplified geometry
```
$ xacro --inorder your_parallel_robot.xacro > your_parallel_robot.urdf
```
Check the URDF file for its correctness
```
$ check_urdf your_parallel_robot.urdf
```
**Convert the URDF file to SDF**
```
$ gz sdf -p your_parallel_robot.urdf > your_parallel_robot.sdf
```
**Open the SDF-file** and look for the **corresponding joint** you want to break the closed loop at (I personally think it is generally best to split the parallel robot into serial chains where the end-effector is the last link, keep one of these connections and then introduce the joints between the other legs and the end-effector by SDF-code injection.).
Now **introduce the other legs of the parallel robot as serial chains but without the final joints to end-effector** (In order to respect URDF's serial chains of course the first chain has to stay connected to the end-effector!). Adapt the syntax that you found in the SDF-file for the joint between the first leg and the end effector to the other final joints by introducing parameters and **copy it into your UDRF-file encapsulated in `<gazebo> ... </gazebo>`** tags (last part of `model/element.xacro` in my code). Furthermore the guides ([1](http://sdformat.org/spec?ver=1.7&elem=joint) and [2](http://sdformat.org/tutorials?tut=spec_model_kinematics)) on the official SDF page might be helpful to understand the SDF-syntax. What is important to know is that the **location of the joint in the parent frame is determined by the location of the child and its initial position**.

## Known issues
Additionally to the given example of a four-bar-linkage I have successfully used this approach with 1R2T and 3R3T cable-driven parallel robots.
It seems though as if there might be some limitations to this approach as discussed in [issue #1](https://github.com/2b-t/closed_loop/issues/1). I have made a couple of test but could not find out yet in which situations it would work and in which it would fail. I will update this section as soon as I know more.
