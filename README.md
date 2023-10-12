# Multiple-Goals-Ordering-Navigation-Algorithm
The repository for [Improving efficiency and cost of ordering algorithms in pathfinding using shell layers](https://www.sciencedirect.com/science/article/abs/pii/S0957417423024508). A scientific journal paper accepted at Expert Systems with Applications in the fields of Robotic Navigation and Artificial Intelligence. 


## MuGONA's Description:


MuGONA is a set of many algorithms integrated together. This integration takes a top-view 2D image map file of an autonomous mobile robot's workspace as input. The input map should contain only black and white coloured areas where white coloured areas represent navigatable passages for the robot while black areas represent blocked passages that the robot can not navigate through.


MuGONA works mainly on finding a near-optimal visiting configuration (order) to visit the provided goal nodes. Then, it connects the ordered goal nodes by navigatable path sections providing sampled path coordinates along the whole path.


The algorithm proceeds to process the generated overall path and smooth it using Cubic-splines to be more appropriate for real-life robot navigation.


The algorithm finally outputs a list of the coordinates of the smoothed (or original) overall path and plots it as a figure.



__Work Flow:__


- Firstly, the algorithm generates equally-spaced path nodes exploiting a 2D map, which are traversable and avoid obstacles. This method allows for the efficient exploration of the entire map while ensuring that the motion of the robot remains smooth and continuous.


- Secondly, The algorithm constructs a well-structured neighboring graph that defines the relationships between the generated nodes themselves and the user-defined nodes. This approach improves the performance of the A* search algorithm and reduces the computational complexity of the path-planning problem.


- Thirdly, the algorithm uses the modified ordering algorithm that can determine a near-optimal visiting order for a starting node and a group of goal nodes. This algorithm is based on the A* search algorithm and Euclidean distance and efficiently determines the best visiting order while avoiding obstacles.


- Lastly, the algorithm applies a path-smoothing technique that ensures the obtained near-optimal path is feasible for robot steering systems. We also generated the total trajectory with respect to the average speed of the robot, providing a complete and efficient solution to the path-planning problem.

![MuGONA's Modules](https://drive.google.com/file/d/10lR0Ci-Z8RnxjidCcc8Qp9lA7WpKv717/view?usp=drive_link)


![MuGONA's Ordering Algorithms](https://drive.google.com/file/d/13yYrlKrWToLqsjOX6rPe78qtyaTj3gde/view?usp=sharing)


## Getting Started

__Installation Instructions for MuGONA__
MuGONA can be set up and used in your own projects with the following steps:

__Prerequisites:__

    Python 3.x
    
    
__Initial Assignments:__



- 1- Open file main.py and go to the MAIN section.
- 2- Insert the name of your own PROCESSED map with its corresponding extension (preferably .png) in the corresponding Variable:

        imageDirectory = 'map.png'

- 2.1- If your own map file is not located in the same folder of main.py, write its directory before the name of the map.
- 2.2- If you do not have a map, use the already provided map file (map.png).
- 3- Assign the desired spacing value in terms of pixels to the corresponding Variable:

        spacing = 40 #pixels

- 4- Assign the desired scale value in terms of meters/pixels to the corresponding variable:

        scale = 1 #meters/pixels

- 5- Assign the average constant speed of the robot in terms of meters/pixels to the corresponding variable:

        constantSpeed = 0.8 #m/s

- 6- Assign the coordinates of the nodes that will be worked on during the execution of MuGONA to the corresponding variable  as a list of tuples, each tuple includes the x and y coordinates of one of the user-defined nodes:

        userDefinedNodes = [(2640, 1120), (300, 300), (5266, 1255), (4314, 154), (646, 3678), (2100, 2325), (2710, 106), (1266, 164), (5000, 3660), (3222, 1775), (2475, 3277), (988, 1972), (2645, 2365), (2972, 2383), (2279, 2383), (4000, 2376), (50, 2426), (4313, 2419), (1040, 2680) , (0, 440), (5240, 640), (920, 2400), (2960, 3640), (2320, 3600), (2200, 3160), (360, 3520), (3560, 2160), (560, 0), (2560, 3000), (5080, 120), (1120, 160), (1600, 1160), (3240, 360), (480, 1640), (4800, 3400), (600, 1960), (320, 3480), (880, 760), (2400, 1080), (4120, 1640), (4200, 1400), (2160, 1000), (800, 280), (4400, 600), (160, 1840), (5280, 1880), (4320, 1400), (1840, 520), (2600, 2920), (1880, 3080), (0, 2120), (800, 3040), (1600, 3680), (3880, 1200), (4560, 120), (760, 3560), (5280, 3080), (3560, 560), (2720, 920), (1240, 840), (1880, 2040), (4040, 880), (400, 1360), (2120, 2080), (4400, 3720), (3160, 3760), (3680, 1320), (960, 2440), (4880, 920), (2400, 800), (0, 3360), (2520, 2560), (1160, 1680), (4400, 440), (1800, 360), (3240, 2160), (440, 3640), (5040, 2840), (1560, 3720), (2560, 960), (3160, 2400), (3760, 2160), (3400, 0), (3280, 400), (5120, 680), (3640, 3720), (1680, 1160), (1240, 0), (3280, 640), (520, 1840), (4880, 880), (3400, 40), (1520, 3440), (1120, 920), (2680, 3600), (2280, 3400), (40, 3360), (2880, 680), (5320, 1240), (2520, 960), (3200, 2520), (720, 720), (640, 1160), (4440, 1200), (4960, 0), (3480, 3640), (40, 1400), (1040, 3160), (2520, 3360), (1520, 3280), (2400, 1120), (5200, 1640), (5360, 1640), (5360, 3040), (360, 2880), (560, 2160), (4400, 320), (1080, 520), (2000, 2360)]

- 6.1- if the scale is 1 meters/pixel, then you can insert the coordinates in terms of meters. However, if the scale is not 1 meters/pixel you should calculate the corresponding pixel coordinates of your nodes.
- 7- Assign the starting node coordinates to the corresponding variable. The starting node should be either present in the generated equally-spaced nodes or stored in the variable 'userDefinedNodes':

        startingNode = (300, 300)

- 8- Assign the coordinates of the goal nodes as a list of tuples to the corresponding variable. The goal nodes should be either present in the generated equally-spaced nodes or stored in the variable 'userDefinedNodes':

        goalNodes = [(5266, 1255), (4314, 154), (646, 3678), (2100, 2325), (2710, 106), (1266, 164), (5000, 3660), (3222, 1775), (2475, 3277), (988, 1972)]


__Running MuGONA:__



- 0- After assigning and checking for the necessary preliminaries, RUN the code.
- 1- At first, MuGONA will generate equally-spaced nodes with respect to the specified spacing and then will show the generated equally-spaced nodes and the user-defined nodes on the map plot.
- 2- Secondly, MuGONA will show the selected starting node and the selected goal nodes on the map plot.
- 3- Thirdly, MuGONA will start ordering the goal nodes seeking the near-optimal visiting configuration. The code may run for a certain period of time until all of the nodes are ordered.
- 4- Fourthly, MuGONA will generate the overall path connecting the ordered goal nodes and show it on the map plot.
- 5- Fifthly, MuGONA will smooth the path using cubic splines and show the smoothed path on the map plot.
- 6- In order for MuGONA to proceed, each plot should be closed after it's inspected.
- 7- The recorded data and workflow information (including the coordinates of the generated overall path) are always printed in the terminal.
