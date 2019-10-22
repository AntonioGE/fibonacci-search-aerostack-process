# FibonacciSearchAerostackProcess
This package creates a process used in [Aerostack](https://github.com/Vision4UAV/Aerostack), a software framework for aerial robotic systems created by [Vision4UAV](https://github.com/Vision4UAV) from Polytechnic University of Madrid. This process can be used for autonomously tunning the position and altitude PID mid-level controllers used in Aerostack. The package uses the Fibonacci search tunning method for tunning two gains of the PID simultaneously.


The algorithm has been based on the work of the following paper: [Real-Time Model-Free Minimum-SeekingAutotuning Method for Unmanned Aerial VehicleControllers Based on Fibonacci-Search Algorithm](https://www.researchgate.net/publication/330369094_Real-Time_Model-Free_Minimum-Seeking_Autotuning_Method_for_Unmanned_Aerial_Vehicle_Controllers_Based_on_Fibonacci-Search_Algorithm), that has been developed by the Institute of Control, Robotics and Information Engineering of the Poznan University of Technology and the Department of Cybernetics, Faculty of Electrical Engineering of the Czech Technical University in Prague.
     
## Installation 

```~/workspace/ros/aerostack_catkin_ws/src/
     -behavior_follow_UAV
 	    -CMakeLists.txt
         -package.xml
         -launch
             -behavior_follow_uav.launch
 		-src
             -include
                 -behavior_follow_uav.h
             -source
                 -behavior_follow_uav.cpp
                 -behavior_follow_uav_main.cpp
```
Hola
