# Behavior/Trajectory Planing for intersection scenario in Griesheim
repository for ADP 151/20
![standard-readme compliant](https://img.shields.io/badge/readme%20style-standard-brightgreen.svg?style=flat-square)](https://github.com/RichardLitt/standard-readme)

## Table of Contents
- [Background](#background)
- [Requirements](#requirements)
- [Usage](#usage)
- [MapParser](#mapparser)
- [Attention](#attention)

## Background

Institute of Automotive Engineering (FZD) is working on the development of autonomous driving. As part of the aDDa 4 Students (Automated Driving Darmstadt for Students) initiative, students should work together on an autonomous vehicle with nine other TU Darmstadt departments. A behavior planner or trajectory planner is an essential part of autonomous driving. This planner should also be implemented in the Project "Verification Validation Methods," demonstrating the data reduction method.

A behavior planner makes decisions such as lane changes, overtaking, etc., on a high level of the autonomous vehicle's software architecture. This module defines a consistent set of rules that are evaluated to select optimal behavior. The possible paths and speed profiles examined at a lower level in trajectory planning are thus restricted. The ADP aims to develop a planner specifically for urban intersection scenarios. Ultimately, the planner should work in the simulation in the urban scenarios without any problems.

In detail, the following work steps must be carried out:
- Research on the state of the art for the behavior / trajectory planner.
- Definition of the requirements for the planner for intersection scenarios.
- Development of the planner based on the requirements.
- Structure of the simulation using e.g. Carla, CarMaker, etc. and implementation of the selected method in ROS.
- Evaluation and comparison of the methods in different scenarios.
- Documentation.

[![framework](/fig/ros_script_ppt.png)

## Requirements
This repository depends on `Ubuntu`, `ROS`, `Eigen` (a third-party library in CPP) and some python libraries (convert msg topic to MATLAB executable data file, not necessary).

### 1. Ubuntu
Since ROS and CarMaker need to run under the ubuntu operating system, this project must be loaded with the `Ubuntu 18.04` system. 
`Two options` are provided below：

#### 1.1 Dual-system with ISO file
Download the 18.04 LTS version of Ubuntu, for desktop PCs and laptops:
-[Ubuntu 18.04 LTS](https://cdimage.ubuntu.com/netboot/18.04/?_ga=2.49800102.1446374243.1614525230-2029365302.1605216805)
-[Create a bootable USB stick on Ubuntu](https://ubuntu.com/tutorials/create-a-usb-stick-on-ubuntu#1-overview)
-[Install Ubuntu desktop](https://ubuntu.com/tutorials/install-ubuntu-desktop#5-prepare-to-install-ubuntu)

#### 1.2 Subsystem in Windows environment
Install a complete Ubuntu terminal environment in minutes on Windows 10 with Windows Subsystem for Linux (WSL). Access the Linux terminal on Windows, develop cross-platform applications, and manage IT infrastructure without leaving Windows.
-[Ubuntu 18.04 LTS Windows-subsystem](https://www.microsoft.com/en-us/p/ubuntu-1804-lts/9n9tngvndl3q?activetab=pivot:overviewtab)

X410 is an X Window server for Windows 10. When you want to use X Window GUI apps on remote servers, simply run X410 and connect to your server via SSH with X11 forwarding. Once connected, just launch your GUI app from the command prompt; it'll pop up and ready for you on Windows 10!
-[X410](https://www.microsoft.com/en-us/p/x410/9nlp712zmn9q?activetab=pivot:overviewtab)
-[Using X410 with WSL2](https://x410.dev/cookbook/)

### 2. ROS
The Robot Operating System (ROS) is a set of software libraries and tools that help you build robot applications. From drivers to state-of-the-art algorithms, and with powerful developer tools, ROS has what you need for your next robotics project. And it's all open source.
-[ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)

### 3. Eigen (a third-party library in CPP)
In this project, trajectory planner depends on an open source matrix calculation library `Eigen`, which provides various interfaces for solving matrix operations. Moreover, Eigen is libraries designed for cross platform(Win/Mac OS/Linux).
It can be downloaded from the following link：
-[Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)

Eigen consists only of header files, hence there is nothing to compile before using them. 

Installing using CMake：
Let's call this directory `source_dir`(where this INSTALL file is).
Before starting, create another directory which we will call `build_dir`.

Do:
  ```
  cd build_dir
  cmake source_dir
  make install
  ```
The "make install" step may require administrator privileges `root`.
You can adjust the installation destination (the "prefix") by passing the -DCMAKE_INSTALL_PREFIX=myprefix option to cmake, as is explained in the message that cmake prints at the end.

If you want to use `find_Package` in CMakeLists .txt for ROS node compiling by yourself, to include these header files automatically,after installation, please check whether the `Eigen` is correctly installed. -> check the path to `FindEigen3.cmake` and create a blank CMakeLists.txt with text below and run this file with `cmake .`
```
cmake_minimum_required(VERSION 3.12)
project(EigenCheck)
find_package(Eigen3)
message(${Eigen3_FOUND})
```
when the eigen3 is correctly installed, in the terminal, flag `1` will be printed out.
### 4. Python3 libraries and dependency check (not necessary)
where Python3 is needed :
- convert the message topic into a mat file for following processing in MATLAB.
- use MapParser in python to generate discrete points list in format of txt or csv, from mapfile `Griesheim.xodr`.

If you are using `pip` for the package management, please check whether you have installed pkg below:
- regex  
- numpy
- matplotlib   
- beautifulsoup4 

Or jsut use command below to install them automatically.

``` 
pip install regex beautifulsoup4 numpy matplotlib
```

If you are using `conda` for the package management, similar to upper, check the package installation.
```
conda install regex numpy matplotlib beautifulsoup4
```

It is highly recommend to create new vitural env for this project, avoid version conflict or unknown errors.
For `pip` users pyenv is user-freiendly and lightweight; for `conda` users, conda iteself does good for it, like ```conda create -n FSM ```

## Usage
The following section briefly describes the specific steps to start each node. All command line operations are executed in `terminal`, which can be opened through `ctrl + alt + T` and `ctrl + shift + T`.

### 1. CarMaker Preparation
- Run roscore firstly
  ```
  roscore
  ```

- Open CarMaker 8.1 with `root`
  ```
  CM-8.1
  ```

- Choose the TestRun project (operation in top menu bar)
  - click `File` -> `Open` -> click `Project` (side menu bar) -> `Griesheim_sim_xxxx`

- Choose the suitable scenario environment (operation in top menu bar)
  - click `Parameters` -> `Scenario / Road` -> click `Load Road file` (side menu bar) -> `Griesheim_v06_xxxx`

- Choose the API Server (operation in top menu bar) based on `Open loop` or `Closed loop`
  - Open loop test:
    - click `Application` -> `Configuration/Status` -> `command` -> select `Car_CMake` -> click `Start & Connect`
  - Closed loop test:
    - click `Application` -> `Configuration/Status` -> `command` -> select `Car_CMake_new` -> click `Start & Connect`

- Now go back to termianl and Do the `ROS Node Activation`

## 2. ROS Node Activation
In this project, we provide different launch files to activate the nodes uniformly. 
Please note that you `only need to perform one of the 2.1 operations as required`.

### 2.1 (a) Open loop Test
Here involves only command line operations executed in `terminal`.

Do：
  ```
  roslaunch cross_fsm cross_fsm.launch
  ```
Do:
  ```
  roslaunch straight_fsm straight_fsm_node.launch
  ```
### 2.1 (b) Closed loop Test
Here involves only command line operations executed in `terminal`.

Do：
  ```
  roslaunch trajectoryPlanner trajectory_planner.launch
  ```

### 2.2 Run Simulation
- go back CarMaker `TestRun` GUI
  - click the green button `Start` -> Simulation runs 


## MapParser

### 1.MaP Introduciton in Griesheim Test Field

The picture below shows the basic road connection in Griesheim.

![Overview](/fig/map_Nr1.jpg)

Griesheim test field contains 24 road segmentions and three junctions in total.

Road ID `1~6`  : out of juncitons, each road contains **two lanes**.

Road ID `7~24` : inside juncitons, each road contains **only one lane**.


#### Junction 1

The picture and table below will show the concrete road connection relationship inside Junciton 1..

![Overview2](/fig/junction_Nr1.jpg)

| Road ID | linking from Road ID | linking to Road ID  |
| :-----:| :----: | :----: |
|Road 19|Road 6|Road 5 |
| Road 24|Road 5 | Road 6 |
| Road 20 |Road 5 | Road 3 |
| Road 21|Road 3 | Road 5 |
| Road 22|Road 3 | Road  6|
| Road 23|Road 6 | Road 3 |


#### Junction 2

The picture and table below will show the concrete road connection relationship inside Junciton 2.

![Overview3](/fig/junction_Nr2.jpg)

| Road ID | linking from Road ID | linking to Road ID  |
| :-----:| :----: | :----: |
|Road 13|Road 4 |Road 5 |
| Road 18 |Road 5| Road 4|
| Road 14|Road 4| Road 2|
| Road 15|Road 2 | Road 4  |
| Road 16 |Road 2| Road 5 |
| Road 17|Road 5 | Road 2 |


#### Junction 3

The picture and table below will show the concrete road connection relationship inside Junciton 3.

![Overview4](/fig/junction_Nr3.jpg)

| Road ID | linking from Road ID | linking to Road ID  |
| :-----:| :----: | :----: |
|Road 7|Road 1 |Road 6 |
| Road 12 |Road 6| Road 1|
| Road 8|Road 1| Road 4|
| Road 9|Road 4| Road 1|
| Road 10|Road 4| Road 6|
| Road 11|Road  6| Road  4|



## Attention
- after download the repository please change the map file path (here is a absolute path), otherwise the core will dump!!

open `intersection/trajectoryPlanner/src/main.cpp`
Change `line 28` and `line 29` as:
  ```
  std::ifstream infile("/[your workspace path]/src/trajectoryPlanner/path_file/interval40cm_right.txt");
  std::ifstream infile("/[your workspace path]/src/trajectoryPlanner/path_file/interval40cm_straight.txt");
  ```


- if you want to change the road marker visualization in Rviz

open `intersection/waypoint_loader/launch/waypoint_loader.launch`
Change `line 4`as:
  ```
  <arg name="mapFileName" default="$(find waypoint_loader)/cfig/[your road marker csv file]" />
  ```

- msgs under vaafo_msg has been modified, it can not use for old rosbag



