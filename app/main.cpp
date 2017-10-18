/**
 * @file main.cpp
 * @brief This project builds a discrete 3-D map from environment information
 * and Implements the A* algorithm to plan the path from Start to Goal Point.
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

/** --Includes--*/
#include <iostream>
#include <vector>
#include <cmath>
#include "../include/Planner.h"
#include "../include/Build_Map.h"

/**
 * @brief main method
 *
 * Takes the World Boundary and Obstacle Data. Sends them to Build a Discrete
 * 3-D Map. This map is sent to the planner which then plans the path from
 * Start to Goal point using A* Planner.
 *
 * @return 0
 */
int main() {
  /**
   * Initialize the x,y,z resolution values.
   * Initialize the Margin Value for Robot Dimensions.
   * Initialize the World Boundary and Obstacles.
   */
  double xy_res = 0.25;  ///< Initialize the x,y resolution.
  double z_res = 0.25;  ///< Initialize the z resolution.
  double margin = 0.2;  ///< Initialize the margin value for robot dimensions.

  /** Initialize the World Boundary in
   * {xmin,ymin,zmin,xmax,ymax,zmax} format.
   */
  std::vector<double> Boundary = { 0.0, -5.0, 0.0, 10.0, 20.0, 6.0 };
  /** Initialize the Obstacles in {xmin,ymin,zmin,xmax,ymax,zmax} format. */
  std::vector<std::vector<double>> Obstacle = {
      { 0.0, 2.0, 0.0, 10.0, 2.5, 1.5 }, { 0.0, 2.0, 4.5, 10.0, 2.5, 6.0 }, {
          0.0, 2.0, 1.5, 3.0, 2.5, 4.5 } };

  /** Create an Instance of the Build_Map Class named Map. */
  Build_Map Map = Build_Map(Boundary, xy_res, z_res, margin);

  /** Store the Discretized World Dimensions in World. */
  std::vector<int> World = Map.World_Dimensions();

  /** Create an Instance of the Planner Class named Plan. */
  Planner Plan = Planner({ World[0], World[1], World[2] });

  /** Add the nodes inside Obstacles to Collision List.  */
  for (const std::vector<double> &v : Obstacle) {
    std::vector<int> Obstacle_Extrema = Map.Build_Obstacle(v);
    for (int Counter_X = Obstacle_Extrema[0]; Counter_X != Obstacle_Extrema[3];
        Counter_X++) {
      for (int Counter_Y = Obstacle_Extrema[1];
          Counter_Y != Obstacle_Extrema[4]; Counter_Y++) {
        for (int Counter_Z = Obstacle_Extrema[2];
            Counter_Z != Obstacle_Extrema[5]; Counter_Z++) {
          Plan.Add_Collision({ Counter_X, Counter_Y, Counter_Z });
        }
      }
    }
  }

  /** Set Heuristic Function to Euclidean or Manhattan (Default Euclidean). */
  Plan.Set_Heuristic(Planner::Manhattan);
  std::cout << "Calculating Shortest Path ... \n";
  std::vector<double> Start = { 0, 0.5, 3 };  ///< Initialize Start Point.
  std::vector<double> Goal = { 3.9, 6.4, 0 };  ///< Initialize Goal Point.

  /** Check whether the Start or Goal points lie outside the World. */
  if ((Start[0] < Boundary[0] || Start[0] > Boundary[3])
      || (Start[1] < Boundary[1] || Start[1] > Boundary[4])
      || (Start[2] < Boundary[2] || Start[2] > Boundary[5])) {
    std::cout << "Start Point Lies Out of Workspace.";
  } else if ((Goal[0] < Boundary[0] || Goal[0] > Boundary[3])
      || (Goal[1] < Boundary[1] || Goal[1] > Boundary[4])
      || (Goal[2] < Boundary[2] || Goal[2] > Boundary[5])) {
    std::cout << "Goal Point Lies Out of Workspace.";
  } else {
    /** If Start and Goal Points are inside the world, find their positions
     * in the discretized world.
     */
    std::vector<int> Start_Node = Map.Build_Node(Start);
    std::vector<int> Goal_Node = Map.Build_Node(Goal);

    /** Plan the Path from Start to Goal using findPath.
     * Get the value in path.
     */
    auto path = Plan.findPath({ Start_Node[0], Start_Node[1], Start_Node[2] },
                              { Goal_Node[0], Goal_Node[1], Goal_Node[2] });

    /** Print the Path */
    std::cout << "X\tY\tZ\n";
    for (auto& coordinate : path) {
      std::vector<int> Discrete_Node = { coordinate.x, coordinate.y, coordinate
          .z };
      std::vector<double> Coordinates = Map.Get_Coordinate(Discrete_Node);
      std::cout << Coordinates[0] << "\t" << Coordinates[1] << "\t"
          << Coordinates[2] << "\n";
    }
  }
  return 0;  ///< Return 0.
}
