# Fibonacci Search Aerostack Process
This package creates a process used in [Aerostack](https://github.com/Vision4UAV/Aerostack), a software framework for aerial robotic systems developed by [Vision4UAV](https://github.com/Vision4UAV) from Polytechnic University of Madrid. This process can be used for autonomously tunning the position and altitude PID mid-level controllers used in Aerostack. The package uses the Fibonacci search tunning method for tunning two gains of the PID simultaneously.


The algorithm has been based on the work of the following paper: [Real-Time Model-Free Minimum-SeekingAutotuning Method for Unmanned Aerial VehicleControllers Based on Fibonacci-Search Algorithm](https://www.researchgate.net/publication/330369094_Real-Time_Model-Free_Minimum-Seeking_Autotuning_Method_for_Unmanned_Aerial_Vehicle_Controllers_Based_on_Fibonacci-Search_Algorithm), that has been developed by the Institute of Control, Robotics and Information Engineering of the Poznan University of Technology and the Department of Cybernetics, Faculty of Electrical Engineering of the Czech Technical University in Prague.

<p align="center">
     <a href="http://www.youtube.com/watch?feature=player_embedded&v=YUOkMmKgKLg
     " target="_blank"><img src="http://img.youtube.com/vi/YUOkMmKgKLg/0.jpg" 
     alt="IMAGE ALT TEXT HERE" width="480" height="360" border="10" /></a>
</p>

## Installation 

1. Move the files of this repository to the directory `~/workspace/ros/aerostack_catkin_ws/src/` so that the file tree looks like it follows:

     ```
     ~/workspace/ros/aerostack_catkin_ws/src/
          FSA_process/ 
               CMakeLists.txt 
               package.xml 
               config.xml 
               src/ 
                    include/ 
                         FSA_process.h 
                    source/ 
                         FSA_process.cpp
                         FSA_process_main.cpp 
               launch/ 
                    FSA_process.launch
     ```
2. Compile Aerostack using the following command:
     ```~/workspace/ros/aerostack_catkin_ws/$ catkin_make```

## Usage

The process can be launched for simulation and real flights. 

### Simulation
1. Activation of ROS using the command `roscore`
2. Activation of the launchers for Gazebo using the commands in the Aerostack folder `launchers`:
     ```
     ./rotors_simulation_agents.sh
     ./rotor_simulation_drivers.sh
     ```
3. Initialization of the Extended Kalman Filter using the `L` key in the monitoring terminal.
4. Take off the drone by using `T` key, activate the Extended Kalman Filter using the `O` key.
5. Activate the position control with `8` key.
6. Activation of the Process by using the following command in a new terminal:
     ```
     $ FSA_process FSA_process.launch --wait drone_id_namespace:=drone7 drone_id_int:=7 my_stack_directory:={AEROSTACK_STACK}
     ```
7. Wait for the tunning for complete (~10 min) and land the drone.

