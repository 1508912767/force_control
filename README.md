## Compliation and build

You also need the following ros packages
```bash
$ sudo apt-get install ros-indigo-ridgeback-*
$ sudo apt-get install ros-indigo-universal-robot
```
if you are getting error for broken packages (most probably due to a wrong version of gazebo), you can use 'aptitude' instead of 'apt-get' which propose a solution and resolve the conflict. 


Finally complie
```bash
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
$ catkin_make
```
* you might need the source the bash file and compie again if the first compliation could not find some of in house dependencies.


For simulator, you can use gazebo7
```bash
$ sudo apt-get install ros-indigo-gazebo7-*
```
You might need to follow [these instructions](http://gazebosim.org/tutorials?tut=install_ubuntu#Alternativeinstallation:step-by-step).

---
---



# Control Architecture

## Kinematics and transformations

Here is a short list of important frames and their usage.

| frame id      | Usage                         |
|---------------|-----------------------------------|
| world                          | Odometry and navigation           |
| ur5_arm_base_link              | Arm pose and twist                |
| base_link                      | Platform pose and twist           |
| robotiq_force_torque_frame_id  | External force applied to the end-effector           |





## Adamittance dynamics
The following figure shows the controller architecture for the admittance control on the robot.

![alt text](fig_control_schematics.png "Control architecture")

The two equations in the center describe the admittance dynamics which compute the desired accelaration for the arm and the platform. These accelerations are integrated in time to acheive the desired velocities for the robot. The low-level velocity controller fullfills these velocities. In the case of platform, the computed velocities can be modified accroding to obstacle avoidance node.



The admittance parameters (shown in purple) are as follows: 

| Variable      | Parameter                         |
|---------------|-----------------------------------|
| M<sub>a</sub> | Desired mass of the arm           |
| D<sub>a</sub> | Desired damping of the arm        |
| D<sub>c</sub> | Desired damping of the coupling   |
| K<sub>c</sub> | Desired Stiffness of the coupling |
| M<sub>p</sub> | Desired mass of the platform      |
| D<sub>p</sub> | Desired damping of the platform   |

These parameters are load from a yaml through the launch file.


### External force
The external is initially measured by the force/torque sensor in its own frame reference. In admittance controller this force is transformed to "ur5_arm_base_link" where the control of arm takes place. To avoid reacting to small forces a deadzone is considered. Moreover, a low-pass filter is used to smooth the measurements. The parameters of the deadzone and the filter can be set from the launch file.

### Higher-level control (Motion planning )
Through a higher level controller, the position of the equilibrium can be can be changed to acheive a desired behavior. Also, F<sub>c</sub> can be used for control purposes; e.g., to compensate for the mass of an object carried by the arm.

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/7BjHhV-BkwE/0.jpg)](https://youtu.be/7BjHhV-BkwE)



# 这里面的用的controller其实和UR之前下载的controller没有什么本质的区别,仅仅是控制模式不一样而已,一个是velocity,而另一个是joint_trajectory,以后需要用到cartesian space的速度可以考虑用这个包!


# 仿真程序
roslaunch ur5_admittance_control admittance_controller.launch

# 真实程序
roslaunch ur5_admittance_control admittance_controller.launch sim:=false F/T_sensor:=true

# 安装错误
因为这个包很多ur的东西,所以很容易产生错误，如果错误无法解决，controller之类的错误，就把catkin_ws环境下的其他包移除，只留force_control的几个包然后，catkin_make就可以了






