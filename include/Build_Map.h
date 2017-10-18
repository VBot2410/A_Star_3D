/**
 * @file Build_Map.h
 * @brief This File contains the declarations of variables and methods for
 * Build_Map class which Discretizes the World using resolution values and
 * marks obstacle positions in the workspace. Converts points from World
 * to Discretized Workspace and vice versa.
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

#ifndef MID_TERM_A_STAR_3D_INCLUDE_BUILD_MAP_H_
#define MID_TERM_A_STAR_3D_INCLUDE_BUILD_MAP_H_

/* --Includes-- */
#include <vector>
#include <functional>
#include <set>
#include "Planner.h"

/**
 * @brief Build_Map class declaration.
 */
class Build_Map {
 public:  ///< Public Access Specifier
  Build_Map(std::vector<double>, double, double, double);  ///< Constructor
  /** Boundary of type double vector, Stores World Boundary Data */
  std::vector<double> Boundary;
  /** World of type integer vector, Stores Discretized World Dimensions */
  std::vector<int> World;
  /** x,y,z Resolutions and Robot Dimensions Margin Data of type double */
  double xy_res, z_res, margin;
  /**
   * @brief World_Dimensions function returns the Discretized World Dimensions
   */
  std::vector<int> World_Dimensions();
  /**
   * @brief Build_Obstacle Creates Obstacle Representations in Discretized
   * Workspace given the Obstacle Blocks in World.
   */
  std::vector<int> Build_Obstacle(std::vector<double>);
  /**
   * @brief Build_Node creates the representation of a point from the world in
   * Discretized Workspace
   */
  std::vector<int> Build_Node(std::vector<double>);
  /**
   * @brief Get_Coordinate creates the representation of a point from location
   * in the Discretized Workspace
   */
  std::vector<double> Get_Coordinate(std::vector<int>);
  virtual ~Build_Map();  ///< Destructor for Class Build_Map
};

#endif  // MID_TERM_A_STAR_3D_INCLUDE_BUILD_MAP_H_
