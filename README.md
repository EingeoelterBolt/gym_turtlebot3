# ros2

# Table of contents
1. [Setup Virtual Machine](#virtualMachine)
2. [Install ROS2 (foxy)](#ros2)
3. [Install Gazebo & Turtlebot3](#gazeboAndTurtlebot)
4. [Run Turtlebot3](#runTurtlebot)
5. [Tips](#tips)
6. [Common Issues](#CommonIssues)
7. [Preconfigured Virtual Machine](#Preconfigured)

## Setup Virtual Machine <a name="virtualMachine"></a>
**NOTE: Using Virtual Machine is not recommended, because of usally bad performance (slower than real time). Instead install Ubuntu directly on your PC! If you still want to use VirtualBox you can follow the steps in this section.** 

1. Install VirtualBox [(Link)](https://www.virtualbox.org/)
2. Download Ubuntu 20.04.2.0 LTS [(Link)](https://ubuntu.com/download/desktop)
3. Create new virtual machine (vm) with Ubuntu (64-bit)
4. After creation make sure to change vm settings:
    * Set used processor cores (more is better)
      * When cores is set to low (e.g. only 1 core) it seems like, that gazebo can't start  
    * Set Graphics memory to 128 MB
    * **IMPORTANT:** make sure under display the 3D acceleration is unchecked 
        * ![image](https://user-images.githubusercontent.com/64859371/113202332-eddffc00-926a-11eb-8f72-fb802f7bd74e.png)
    * Under storage bind downloaded ubuntu-iso file [(Youtube Link)](https://youtu.be/QbmRXJJKsvs?t=424) 

## Install ROS2 (foxy) <a name="ros2"></a>
To install ROS2 (foxy) follow instructions from the official documentation [(Doc Link)](https://docs.ros.org/en/foxy/Installation/Linux-Install-Debians.html).
Some remarks about the installation:

1. While setting up the environment [(Doc Link)](https://docs.ros.org/en/foxy/Installation/Linux-Install-Debians.html#environment-setup) call ```echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc``` to avoid to call ```source /opt/ros/foxy/setup.bash``` on every terminal new start see also: [(Doc Link)](https://docs.ros.org/en/foxy/Tutorials/Configuring-ROS2-Environment.html#add-sourcing-to-your-shell-startup-script)
2. Don't forget to install the optional argcomplete package [(Doc Link)](https://docs.ros.org/en/foxy/Installation/Linux-Install-Debians.html#install-argcomplete-optional)
    * ```sudo apt install -y python3-argcomplete```
3. Prefer Desktop install over ROS-Base version. It loads usefull GUI tools [(Doc Link)](https://docs.ros.org/en/foxy/Installation/Linux-Install-Debians.html#install-ros-2-packages) 
    * ```sudo apt install ros-foxy-desktop```

## Install Gazebo & Turtlebot3 <a name="gazeboAndTurtlebot"></a>
Follow [(Youtube Link)](https://youtu.be/8w3xhG1GPdo?t=10) and [(Doc Link)](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup) (make sure Foxy is selected)

## Run Turtlebot3 <a name="runTurtlebot"></a>
[(Doc Link)](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)
(make sure Foxy is selected)

## Tips <a name="tips"></a>
### Handle multiple terminals
When working with ROS2 multiple terminals commenly used. To handle multiple terminals the tool "terminator" is recommanded to install call ```sudo apt-get install terminator```  a demo can be seen on [(Youtube Link)](https://youtu.be/dq4RlgxObN4?t=132)

### Export default Turtlebot3
To avoid selecting turtlebot3 model on every start call ```echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc```.<br/>
Available models [(Doc Link)](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation):
  * burger
  * waffle
  * waffle_pi 

## Common Issues <a name="CommonIssues"></a>
### Copy & Paste doesn't work (from host to vm) 
To fix this issue follow this video [(Youtube Link)](https://youtu.be/8MiPmL0YaJk?t=94) 

### Gazebo shows black window 
This post [(Link)](https://www.sezercoban.com/gazebo-black-screen-problem-for-urdf/) from Sezer suggests to make sure under display the 3D acceleration is unchecked ![image](https://user-images.githubusercontent.com/64859371/113202332-eddffc00-926a-11eb-8f72-fb802f7bd74e.png)

### Preconfigured Virtual Machine
A preconfigured Virtual Machine can be found on: [Google Drive](https://drive.google.com/drive/folders/1Fu4Gzck-nOmXErTDnHrkFz91u1w0GiUj?usp=sharing)<br/>
**Credentials:**
  * username: **ki**
  * password: **ki**

## Don't use rospy instead use rclpy
With ros2 rospy doesnt work anymore. Instead use rclpy https://docs.ros2.org/latest/api/rclpy/api.html

infos unter: http://design.ros2.org/articles/generated_interfaces_python.html

