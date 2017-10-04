# Implementation of A* Planning Algorithm in 3D Environment
[![Build Status](https://travis-ci.org/VBot2410/A_Star_3D.svg?branch=master)](https://travis-ci.org/VBot2410/A_Star_3D)
[![Coverage Status](https://coveralls.io/repos/github/VBot2410/A_Star_3D/badge.svg?branch=master)](https://coveralls.io/github/VBot2410/A_Star_3D?branch=master)
---

## Overview
A* is a widely used algorithm for pathfinding in 2-D environments. This project takes the planning algorithm one step further and applies it to a 3-Dimensional environment.
This project's objective is to buid a planning module for Aerial Vehicles. Provided with the world boundary and Obstacle location data, the module provides the optimal path from start point to the goal by avoiding all the obstacles.

## Description
The World boundary and obstacle data is provided in the (xmin,ymin,zmin,xmax,ymax,zmax) format. The module then uses this data along with the robot dimensions information to discretize the world and build a 3-D Configurtion Space. This C-Space is used to plan the Robot's path using A* Search.
#### A* Search Algorithm
A* is an algorithm that searches for smallest cost path among all possible paths from star to goal. At each iteration of its planning stage, A* expands the node with the least cost-to-start until it finds the optimal path from start to goal.

## Build Instructions
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
