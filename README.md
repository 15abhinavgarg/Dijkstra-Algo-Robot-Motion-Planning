# Dijkstra-Algo-for-Path-Planning
Implementation of Dijkstra algorithm to determine the shortest path 

The python program uses the below library

1. Numpy
2. OpenCV
3. Matplotlib
4. Heapq

To execute the python file make sure you have all the libraries installed and enter the below command in terminal

=> python Dalgo.py

The program takes input cordinate of start and goal point and shows animation to reach the it using Dijkstra Algorithm. In case the input points are on obstacle or out of bound of the map, the program terminates with the appropriate message

The obstacle course was given in the assignment, and by using opencv.circle I was able to draw the circle obstacle. similarly using algebraic equation, trignometry and geometry points of other obstacles were detected and plotted.

The function that plots obstacle has been named draw_obstacles.

Heap priority queue and dictionary has been used to fetch the lowest cost to come value. Dictionary was used to maintain the updated cost to come value as the values in priority queue could not be manipulated.

The function which defines all possible steps is named allpossiblesteps() and it assigns the cost function of each step.

Opencv was used to animate the nodes evaluated and to draw the shortest path.

Video has been attached for the animation

