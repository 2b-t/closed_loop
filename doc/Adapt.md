# Closed loop parallel robots in ROS and Gazebo

Author: [Tobit Flatscher](https://github.com/2b-t) (April 2020)



## 2. Adapt it to your robot

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

