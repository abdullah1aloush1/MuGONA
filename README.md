# Multiple-Goals-Ordering-Navigation-Algorithm
Multiple-Goals-Ordering-Navigation-Algorithm



Definition:




MuGONA is a combined algorithm that takes a top-view 2D image map file of the workspace as input. The map should contain only black and white coloured areas where white coloured areas represent navigatable passages for the robot while black areas represent blocked passages that the robot can not navigate through.
The algorithm works on finding a near-optimal visiting configuration (order) to visit the provided goal nodes. Then, it connects the ordered goal nodes by navigatable path sections providing sampled path coordinates along the whole path.
The algorithm proceeds to process the generated overall path and smooth it using Cubic-splines to be more appropriate for real-life robot navigation.
The algorithm finally outputs a list of the coordinates of the smoothed (or original) overall path and plots it as a figure.



Work Flow:



Firstly, the algorithm generates equally-spaced path nodes exploiting a 2D map, which are traversable and avoid obstacles. This method allows for the efficient exploration of the entire map while ensuring that the motion of the robot remains smooth and continuous.

Secondly, The algorithm constructs a well-structured neighboring graph that defines the relationships between the generated nodes themselves and the user-defined nodes. This approach improves the performance of the A* search algorithm and reduces the computational complexity of the path-planning problem.

Thirdly, the algorithm uses the modified ordering algorithm that can determine a near-optimal visiting order for a starting node and a group of goal nodes. This algorithm is based on the A* search algorithm and Euclidean distance and efficiently determines the best visiting order while avoiding obstacles.

Lastly, the algorithm applies a path-smoothing technique that ensures the obtained near-optimal path is feasible for robot steering systems. We also generated the total trajectory with respect to the average speed of the robot, providing a complete and efficient solution to the path-planning problem.




User Guide:


Initial Assignments:



1- Open file main.py and go to the MAIN section.
2- Insert the name of your own PROCESSED map with its corresponding extension (preferably .png) in the variable 'imageDirectory'.
2.1- If your own map file is not located in the same folder of main.py, write its directory before the name of the map.
2.2- If you do not have a map, use the already provided map file (map.png).
3- Assign the desired spacing value in terms of pixels to the variable 'spacing'.
4- Assign the desired scale value in terms of meters/pixels to the variable 'scale'.
5- Assign the average constant speed of the robot in terms of meters/pixels to the variable 'constantSpeed'.
6- Assign the coordinates of the nodes that will be worked on during the execution of MuGONA to the variable 'userDefinedNodes' as a list of tuples, each tuple includes the x and y coordinates of one of the user-defined nodes.
6.1- if the scale is 1 meters/pixel, then you can insert the coordinates in terms of meters. However, if the scale is not 1 meters/pixel you should calculate the corresponding pixel coordinates of your nodes.
7- Assign the starting node coordinates to the variable 'startingNode'. The starting node should be either present in the generated equally-spaced nodes or stored in the variable 'userDefinedNodes'.
8- Assign the coordinates of the goal nodes as a list of tuples to the variable 'goalNodes'. The goal nodes should be either present in the generated equally-spaced nodes or stored in the variable 'userDefinedNodes'.


Running MuGONA:



0- After assigning and checking for the necessary preliminaries, RUN the code.
1- At first, MuGONA will generate equally-spaced nodes with respect to the specified spacing and then will show the generated equally-spaced nodes and the user-defined nodes on the map plot.
2- Secondly, MuGONA will show the selected starting node and the selected goal nodes on the map plot.
3- Thirdly, MuGONA will start ordering the goal nodes seeking the near-optimal visiting configuration. The code may run for a certain period of time until all of the nodes are ordered.
4- Fourthly, MuGONA will generate the overall path connecting the ordered goal nodes and show it on the map plot.
5- Fifthly, MuGONA will smooth the path using cubic splines and show the smoothed path on the map plot.
6- In order for MuGONA to proceed, each plot should be closed after it's inspected.
6- The recorded data and workflow information (including the coordinates of the generated overall path) are always printed in the terminal.
