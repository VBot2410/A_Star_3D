/**
 * @file Build_Map.cpp
 * @brief This File contains the code for Build_Map class which Discretizes
 * the World using resolution values and marks obstacle positions in the
 * workspace. Converts points from World to Discretized Workspace and vice versa.
 *
 * @author Vaibhav Bhilare
 * @copyright 2017, Vaibhav Bhilare
 *
 * MIT License
 * Copyright (c) 2017 Vaibhav Bhilare
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* --Includes-- */
#include <iostream>
#include <cmath>
#include <vector>
#include "../include/Build_Map.h"

/**
 * @brief Constructor for Build_Map Class
 * @param Boundary has type double vector & {xmin,ymin,zmin,xmax,ymax,zmax}format
 * @param xy_res has type double and stores x,y resolution
 * @param z_res has type double and stores z resolution
 * @param margin has type double and stores the margin value for Robot Dimensions
 * @param World has type double vector and stores World Dimensions
 */
Build_Map::Build_Map(std::vector<double> _Boundary, double _xy_res,
                     double _z_res, double _margin)
    : Boundary(_Boundary),
      xy_res(_xy_res),
      z_res(_z_res),
      margin(_margin) {
  int World_X = (Boundary[3] - Boundary[0]) / xy_res;  ///< X Dimension
  int World_Y = (Boundary[4] - Boundary[1]) / xy_res;  ///< Y Dimension
  int World_Z = (Boundary[5] - Boundary[2]) / z_res;  ///< Z Dimension
  World = {World_X, World_Y, World_Z};
}

/**
 * @brief World_Dimensions function returns the Discretized World Dimensions
 * @return World of type integer vector & contains Discretized World Dimensions
 */
std::vector<int> Build_Map::World_Dimensions() {
  return World;
}

/**
 * @brief Build_Obstacle Creates Obstacle Representations in Discretized
 * Workspace given the Obstacle Blocks in World.
 * @param v has type double vector & contains obstacle boundary information
 * @param Obstacle_Extrema has type double vector & contains the Obstale
 * representation in Discretized Workspace
 * @return Obstacle_Extrema of type double vector
 */
std::vector<int> Build_Map::Build_Obstacle(std::vector<double> v) {
  int minx = std::max(
      static_cast<int>(std::floor(((v[0] - Boundary[0] - margin) / xy_res))),
      0);
  int miny = std::max(
      static_cast<int>(std::floor(((v[1] - Boundary[1] - margin) / xy_res))),
      0);
  int minz = std::max(
      static_cast<int>(std::floor(((v[2] - Boundary[2] - margin) / z_res))), 0);
  int maxx = std::min(
      static_cast<int>(std::ceil(((v[3] - Boundary[0] + margin) / xy_res))),
      World[0]);
  int maxy = std::min(
      static_cast<int>(std::ceil(((v[4] - Boundary[1] + margin) / xy_res))),
      World[1]);
  int maxz = std::min(
      static_cast<int>(std::ceil(((v[5] - Boundary[2] + margin) / z_res))),
      World[2]);
  std::vector<int> Obstacle_Extrema = { minx, miny, minz, maxx, maxy, maxz };
  return Obstacle_Extrema;
}

/**
 * @brief Build_Node creates the representation of a point from the world in
 * Discretized Workspace
 * @param Discrete has type double vector & contains x,y,z values of a point
 * in World.
 * @param Built_Node has type integer vector & contains representation of a
 * point in Discretized Workspace
 * @return Built_Node of type integer vector
 */
std::vector<int> Build_Map::Build_Node(std::vector<double> Discrete) {
  int X_Init = (Discrete[0] - Boundary[0]) / xy_res;
  int Y_Init = (Discrete[1] - Boundary[1]) / xy_res;
  int Z_Init = (Discrete[2] - Boundary[2]) / z_res;
  std::vector<int> Built_Node = { X_Init, Y_Init, Z_Init };
  return Built_Node;
}

/**
 * @brief Get_Coordinate creates the representation of a point from location
 * in the Discretized Workspace
 * @param Node_Init has type integer vector & contains representation of a
 * point in Discretized Workspace
 * @param Coordinates_Generated has type double vector and contains x,y,z
 * values of a point in World.
 * @return Coordinates_Generated of type double vector
 */
std::vector<double> Build_Map::Get_Coordinate(std::vector<int> Node_Init) {
  double X_Coordinate = Boundary[0] + (Node_Init[0] * xy_res);
  double Y_Coordinate = Boundary[1] + (Node_Init[1] * xy_res);
  double Z_Coordinate = Boundary[2] + (Node_Init[2] * z_res);
  std::vector<double> Coordinates_Generated = { X_Coordinate, Y_Coordinate,
      Z_Coordinate };
  return Coordinates_Generated;
}

/**
 * @brief Destructor of Build_Map Class
 */
Build_Map::~Build_Map() {
}
