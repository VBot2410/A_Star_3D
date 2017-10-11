# Implementation of A* Planning Algorithm in 3D Environment
[![Build Status](https://travis-ci.org/VBot2410/A_Star_3D.svg?branch=master)](https://travis-ci.org/VBot2410/A_Star_3D)
[![Coverage Status](https://coveralls.io/repos/github/VBot2410/A_Star_3D/badge.svg?branch=master)](https://coveralls.io/github/VBot2410/A_Star_3D?branch=master)
---

## Overview
A* is a widely used algorithm for pathfinding in 2-D environments. This project takes the planning algorithm one step further and applies it to a 3-Dimensional environment.<br />
This project's objective is to buid a planning module for Aerial Vehicles. Provided with the world boundary and Obstacle location data, the module provides the optimal path from start point to the goal by avoiding all obstacles.<br />
Development of this project follows **Solo Iterative Process (SIP)**  model in Software Engineering.<br />
Tested on Ubuntu 16.04 LTS with GCC 5.4.0.


## Description
The World boundary and obstacle data is provided in the (xmin,ymin,zmin,xmax,ymax,zmax) format. The module then uses this data along with the robot dimensions information to discretize the world and build a 3-D Configurtion Space. This C-Space is used to plan the Robot's path using A* Search.
#### A* Search Algorithm
A* is an algorithm that searches for lowest cost path among all possible paths from start to goal. What A* algorithm does is that at each step it picks the node from open list according to the value *f*, which is sum of two other parameters *g* and *h*. <br />
Here, *g* value of a node is the cost to move from start to that node and *h* value is the estimated movement cost to move from that given node to the final destination.There are many ways to calculate *h* and this project provides an option to select between **Euclidean Distance** and **Manhattan Distance**.<br /> Obstacle nodes have the cost of infinity to travel through them and hence, the planner avoids these nodes in path planning.<br />
Typical implementations of A* use a priority queue to perform the repeated selection of minimum (estimated) cost nodes to expand. This priority queue is known as the open list. At each step of the algorithm, the node with the lowest *f(x)* value is removed from the queue, the *f* and *g* values of its neighbors are updated accordingly, and these neighbors are added to the queue. The algorithm continues until a goal node has a lower *f* value than any node in the queue (or until the queue is empty). The *f* value of the goal is then the length of the shortest path, since *h* at the goal is zero in an admissible heuristic.

## Solo Iterative Process (SIP)
1. Iteration 1:
   - Planning of the Project Structure and Repository Setup.
   - Setup Travis and Coveralls.
2. Iteration 2:
   - Build the Unit Testing Environment and Class Stubs.
   - Implementation of Map Builder and A* Planner with Fully working Path Planner.
   - Release Version 1.0.
3. Iteration 3:
   - Add Manhattan Distance Heuristic.
   - Explore the Possibility of 3-D Plots.
   - Release Version 2.0.
4. Iteration 4:
   - Documentation.
   - Code Optimization.

Product and Iteration Backlogs can be found [Here](https://docs.google.com/spreadsheets/d/1OG3C1JBhDQzr5JbrNKvgw2EXLqxJdgZZMIk44BRrPIw/edit?usp=sharing)

UML Diagrams can be found [Here](https://github.com/VBot2410/A_Star_3D/tree/master/UML)

## Status
~~Skeleton Code and Initial UML Diagrams.~~<br />
~~Implementation of Map Builder from Environment Boundary and Obstacle Data.~~<br />
~~Implementation of A* Planner.~~<br />
~~Release Version 1.0 of the Module.~~<br />
**Version 1.0 is Now Online.**

## To Do (Version 2.0)
Explore Possibility of 3-D Plots.<br />
Add Manhattan Distance Heuristic Function.<br />
Doxygen Comments and Documentation.<br />

## Build Instructions (Tested on Ubuntu 16.04 LTS with GCC 5.4.0.)
```
git clone --recursive https://github.com/VBot2410/A_Star_3D
cd <path to repository>
mkdir build
cd build
cmake ..
make

Run tests: ./test/A_Star-test
Run program: ./app/A_Star-app
```
## License
This Project is Licensed under the MIT License. Copy of the License Can be Found [Here](https://github.com/VBot2410/A_Star_3D/blob/master/LICENSE)

MIT License

Copyright (c) 2017 Vaibhav Bhilare
```
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
