# FibonacciSearchAerostackProcess
This package creates a process used in <a href="https://github.com/Vision4UAV">Aerostack</a>, a software framework for aerial robotic systems created by <a href="https://github.com/Vision4UAV/Aerostack">Vision4UAV</a>. This process can be used for autonomously tunning the position and altitude PID mid-level controllers used in Aerostack. The package uses the Fibonacci search tunning method for tunning two gains of the PID simultaneously.
[Aerostack](https://github.com/Vision4UAV)

The algorithm has been based on the work of the following paper: <a href="https://www.researchgate.net/publication/330369094_Real-Time_Model-Free_Minimum-Seeking_Autotuning_Method_for_Unmanned_Aerial_Vehicle_Controllers_Based_on_Fibonacci-Search_Algorithm">Real-Time Model-Free Minimum-SeekingAutotuning Method for Unmanned Aerial VehicleControllers Based on Fibonacci-Search Algorithm</a>, that has been developed by the Institute of Control, Robotics and Information Engineering of the Poznan University of Technology and the Department of Cybernetics, Faculty of Electrical Engineering of the Czech Technical University in Prague.
     
## Installation 

<pre><code> ~/workspace/ros/aerostack_catkin_ws/src/
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
</code></pre>
