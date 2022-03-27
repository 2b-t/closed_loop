# Closed loop parallel robots in ROS and Gazebo with SDF-code injection - Four-bar linkage example

Author: [Tobit Flatscher](https://github.com/2b-t) (April 2020)



## 0. Overview

This is a workaround on how to **include parallel robots into Robot Operating System ROS (URDF) and GazeboSim (SDF) by using Xacro** without having to retype the geometry manually twice for the URDF and the SDF-files. This works by injecting SDF code into the URDF file.

### 0.1 The issue
The Unified Robot Description Format (UDRF) only supports serial chains (meaning a parent may have multiple child nodes but a child only a single parent node) and therefore can't describe parallel robots due to their graph structure with closed loops (two parents for at least one link). Simulation with Gazebo on the other hand supports parallel geometries such as the Simulation Description Format (SDF). As a consequence both files are often written manually, making it error prone in case the geometry changes.
A [previous workaround](https://github.com/wojiaojiao/pegasus_gazebo_plugins) (that is anyways more complicated) did not work for me, so I had to come up with my own solution. 
In this workaround **closed loops are broken up into serial chains** and the **missing joints** are supplied to only Gazebo with corresponding instructions in **SDF**. Furthermore it introduces **XML macros with Xacro** to reduce code redundancy. In the process a single UDRF that can be saved on the ROS parameter server is generated that still contains the SDF to **automatically recover the full parallel geometry in Gazebo**.

### 0.2 The files

The files simulate a simple [four-bar linkage](https://en.wikipedia.org/wiki/Four-bar_linkage) similar to the example in SDF by [The Construct](https://youtu.be/hglRGiNHRno) that collapses to the side under gravity.

- The [`/doc`](./doc) folder contains additional documentation:
  - [`/doc/Adapt.md`](./doc/Adapt.md) describes how you can adapt the same principle to your parallel robot
  - [`/doc/Docker.md`](./doc/Docker.md) describes how the supplied Docker can be used
- The [`/docker`](./docker) folder contains several Docker and Docker-Compose configurations
- [`/launch/closed_loop.launch`](./launch/closed_loop.launch) is the *launch-file* for the ROS parameter server and loads the geometry into Gazebo
- The [`/model`](./model) folder contains the Xacro URDF files:
  - [`/model/closed_loop.xacro`](./model/closed_loop.xacro) is the main file that *loads the components located in the other two Xacro files*
  - [`/model/parameters.xacro`](./model/parameters.xacro) contains the *geometry parameters* such as height, depth, width, mass and macros for the inertial matrix
  - [`/model/element.xacro`](./model/element.xacro) contains macros for a single element and *creates the entire geometry*
- The hidden folders [`/.devcontainer`](./.devcontainer) and [`/.vscode`](./.vscode) contain several configuration files for Visual Studio Code

## 1.1 Launch the solution

You can either download this repository directly and place it as a package into an existing ROS workspace or used the supplied Docker as discussed in more detail in [`/doc/Docker.md`](./doc/Docker.md).

**Rebuild your workspace** (assuming you named it `closed_loop_ws` and placed it inside `home`) with

```shell
$ cd ~/closed_loop_ws
$ catkin build
```
**Launch Gazebo** by typing

```shell
$ source /devel/setup.bash
$ roslaunch gazebo_ros empty_world.launch
```
and my corresponding **launch file** that launches the ROS parameter server and loads the UDRF-model into GazeboSim with
```shell
$ source /devel/setup.bash
$ roslaunch closed_loop closed_loop.launch
```
This should bring up a simple four-bar linkage in Gazebo that collapses to the side.

## 2. Adapt it to your robot

For more information on how to adapt this principle to your robot have a look at the **file [`/doc/Adapt.md`](./doc/Adapt.md)**.

### 2.1 Known issues

Additionally to the given example of a four-bar-linkage I have successfully used this approach with 1R2T and 3R3T cable-driven parallel robots. It seems though as if there might be some limitations to this approach as discussed in [issue #1](https://github.com/2b-t/closed_loop/issues/1). I have made a couple of test but could not find out yet in which situations it would work and in which it would fail.
