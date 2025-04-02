# Implementation for EduBot for AE4324 Physical Interaction for Aerial and Space Robots (2024/25 Q3)

All setup should be pretty identical to specified for the usual edubot installation guide, except instead of installing edubot you clone from this repository. This is done as follows:

```
git clone --recurse-submodules https://github.com/Taouuw/robotics2.git
```



I added an `instant_setup.sh` file which can be sourced to do all steps at the same time:

```
source instant_setup.sh     
# runs source /opt/ros/humble/setup.bash
# if you are using jazzy change this within the file
# runs source cpp_impl/install/setup.bash
# you should then be presented with several options for which you can select a number:
# you can either start in rviz sim, or start hw drivers
```



In another terminal you can then run the python implementation version.

All custom implementation are done in python.

```
source instant_setup.sh -p     # python implementation instead of cpp
# sources
# runs source python_impl/install/setup.bash
# you should then be presented with several options for which you can select a preferred number:
# you can run all custom made trajectories for the exercises here
```



Custom python implementations are generally located in `python_impl/src/controllers/controllers/custom/`

As for the files here:

`robot.py` contains basic implementations for robot IK calculations, including the classes `Point` for a point in 3D and `Robot` for a representation of the robotic arm, which stores the lengths of the links angle limits,

There is a `Robot.inverses(point : Point) -> list[RobotAngles]` method which does inverse kinematics on a point and returns a list of possible robot joint space configurations that solve for it. These joint space configurations are ordered in ascending order based on which configuration has the largest margin to the angle limits.



`robot_controller.py` builds upon the basic structures defined in `robot.py`

It defines a `RobotController(Node)` class which is built in a similar way and has similar functionality to `ExampleTraj(Node)` provided.



`RobotController(Node)` offers a higher level interface for control. You create one by providing it a `Robot` instance for representing the robot its controlling and a `Schedule` class instance which uses waypoints to rerpresent a time parametrized function of EE target positions.

You can set its `mode` to switch between "jacobian velocity control" or standard control.

0.  means standard control (default)

1.  is direct jacobian velocity control: Reads the desired velocity from the schedule directly and thus accumulates error. Is unstable 

2.  reads desired velocity from difference between current position and target position, and thus is stable.



`DemoController(Node)` is a lower level interface for control. Also like `ExampleTraj(Node)`







New entry points for the new custom trajectories are added in:

`python_impl/src/controllers/controllers/custom_traj.py`



Additionally, waypoint data for the flame trajectory is stored in: `python_impl/src/controllers/controllers/data`



And changes were made to `python_impl/src/controllers/setup.py`to include the new trajectories as entry points











# EduBot: The 4-DoF manipulator for Education

This repository contains the drivers, visualization features and a simple simulation for the [Lynxmotion AL5A Arm](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/servo-erector-set-robots-kits/ses-v1-robots/ses-v1-arms/al5a/), all of which are implemented using the [ROS2](https://docs.ros.org/en/humble/index.html) middleware.

## Installation

### Pre-requisites

To compile the pre-requisites are `ROS 2` and `boost`.

#### For Ubuntu 22.04 users: Install ROS 2 Humble:

ROS 2 Humble (LTS) can be installed for ubuntu (>= 22.04) as explained in the [ROS 2 documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). 
Additional prerequisites are found [here](#for-virtual-machine-users) if you're using a Virtual Machine.
The easiest way is to add the sources and install via apt

        # Add the ROS 2 GPG key with apt.
        sudo apt update && sudo apt install curl -y
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg 
    
        # Add the repository to your sources list.
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
        # Update the sources
        sudo apt update && sudo apt upgrade
    
        # Finally install ros and ros-dev tools
        sudo apt install ros-humble-desktop ros-dev-tools

Make sure the additional ros libraries are installed

        sudo apt install ros-humble-xacro ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui
        # Install pip if not yet installed
        sudo apt-get install python3-pip
        # Install python pkg dependency
        pip install catkin_pkg

### For Ubuntu 24.04 users Install ROS 2 Jazzy:

ROS 2 Jazzy (LTS) can be installed for ubuntu (>= 24.04) as explained in the [ROS 2 documentation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html). 
Additional prerequisites are found [here](#for-virtual-machine-users) if you're using a Virtual Machine.
The easiest way is to add the sources and install via apt

        # Add the ROS 2 GPG key with apt.
        sudo apt update && sudo apt install curl -y
        zsudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    
        # Add the repository to your sources list.
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
        # Update the sources
        sudo apt update
    
        # Finally install ros and ros-dev tools
        sudo apt install ros-dev-tools ros-jazzy-desktop

Make sure the additional ros libraries are installed

        sudo apt install ros-jazzy-xacro ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui
        # Install pip if not yet installed
        sudo apt-get install python3-pip
        # Install python pkg dependency
        pip install catkin_pkg

The required `boost` libraries are installed via

        sudo apt install libboost-all-dev

Finally, clone this repository recursively

        git clone --recurse-submodules https://github.com/bioMorphic-Intelligence-Lab/edubot

To allow the `ROS` programm to access the USB port connected to the eduBot you will also have to add the current user to the dialout group

        sudo usermod -a -G dialout $USER

### Compilation

The repository contains two folders: `cpp_impl` and `python_impl`. The former contains the driver, visualization, simulation and a simple control example of the robot while the latter only contains the control example. 
You can choose which language you would like to write your controller in using the code provided as an initial guideline.
To compile each of the packages navigate into the folder, source your `ros` installation and call the `colcon` compilation, e.g.

        cd cpp_impl
        # For ROS 2 Humble users
        source /opt/ros/humble/setup.bash
        # For ROS 2 Jazzy users
        source /opt/ros/jazzy/setup.bash
        colcon build

### Running

Once the package is compiled and sourced via

      source cpp_impl/install/setup.bash

You can run start the simulation or the driver for the robot with the following commands

| Command                             | Effect                                                                                              |
| ----------------------------------- | --------------------------------------------------------------------------------------------------- |
| `ros2 launch edubot sim.launch.py`  | Launches the simulation and `rviz` visualization                                                    |
| `ros2 launch edubot rviz.launch.py` | Launches the `rviz` visualization and a joint position interface which lets you play with the robot |
| `ros2 run edubot robot_hw`          | Starts the Hardware driver for the robot                                                            |
| `ros2 run controllers example_traj` | Starts the an controller that commands a periodic example trajectory                                |

### For Virtual Machine Users

Download and install both **VirtualBox** and **VirtualBox Extension Pack** from [their website](https://www.virtualbox.org/wiki/Downloads). Ensure that both have the same versions. If you have a virtual machine already running, first power it off before installing the Extension Pack and proceeding.

Go to **Virtual Machine Settings > USB**, check **Enable USB Controller** and the corresponding USB type. In my case it was **USB 3.0 (xHCI) Controller**. 

Connect the (1) robot's USB connector, (2) power the robot motors, and then (3) power on virtual machine. Every time you open a new terminal, source your `ros` installation and call the `colcon` compilation, as stated above.

Verify that the virtual machine can read the USB by listing files and directories that starts with **ttyUSB-**

        ls -l /dev/ttyUSB*

If you get a **permission denied** error such as [this](https://support.termius.com/hc/en-us/articles/6325078649753-When-trying-to-make-a-serial-connection-I-get-a-Permission-denied-error#:~:text=Most%20'Permission%20denied'%20error%20messages,identify%20the%20serial%20port%20path.&text=The%20next%20step%20is%20to,running%20the%20command%20provided%20below.) add your user account to the appropriate group by

        sudo usermod -a $USER -G dialout

Log in and log out user account and then try again. Don't forget to source.

**Important**: If the USB still cannot be read, unplug everything and kill all ROS processes, (1) power the virtual machine, (2) power ON the robot emergency switch, (3) plug in the USB connector, (4) plug in power for motors.

Example implementation can be found [here](https://www.youtube.com/watch?v=h-EOHbVqsJg).

## Issues

If any issues with this software occur, please don't hesitate to use the [Issues](https://github.com/BioMorphic-Intelligence-Lab/edubot/issues) pane within this repository.

Good Luck!


