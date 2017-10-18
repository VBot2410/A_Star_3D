/**
 * @file A_Star_Test.cpp
 * @brief Unit tests for Implementation of Build_Map and Planner Classes.
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
#include <gtest/gtest.h>
#include <vector>
#include "../include/Build_Map.h"
#include "../include/Planner.h"

double xy_res = 1;  ///< Initialize the x,y resolution to 1
double z_res = 1;  ///< Initialize the z resolution to 1
double margin = 0.1;  ///< Initialize the margin to 0.1

/** Initialize the World Boundary */
std::vector<double> Boundary = { 0.0, 0.0, 0.0, 10.0, 10.0, 10.0 };

/** Initialize the Obstacles */
std::vector<std::vector<double>> Obstacle = { { 5.0, 0.0, 0.0, 5.0, 10, 10 } };

/** Create an instance of Build_Map Class */
Build_Map Map = Build_Map(Boundary, xy_res, z_res, margin);

/** Store world Dimensions in World. */
std::vector<int> World = Map.World_Dimensions();

/** Create an instance of Planner Class */
Planner Plan = Planner({ World[0], World[1], World[2] });
std::vector<double> Start = { 0, 0, 0 };  ///< Set Start Point to (0,0,0)
std::vector<double> Goal = { 2, 2, 2 };  ///< Set Goal Point to (2,2,2)

/** Get positions of Start and Goal points in Discretized Workspace. */
std::vector<int> Start_Node = Map.Build_Node(Start);
std::vector<int> Goal_Node = Map.Build_Node(Goal);

/** Plan the Shortest Path from Start to Goal. Store in path. */
auto path = Plan.findPath({ Start_Node[0], Start_Node[1], Start_Node[2] }, {
                              Goal_Node[0], Goal_Node[1], Goal_Node[2] });

/**
 * @brief Unit Test to Check the Discretized World Dimensions
 */
TEST(Build_Map, World_Dimensions_Check) {
  std::vector<int> World = Map.World_Dimensions();
  std::vector<int> Test_World = { 10, 10, 10 };
  ASSERT_EQ(World, Test_World);
}

/**
 * @brief Unit Test to Check whether the Obstacles are Created at correct point
 */
TEST(Build_Map, Obstacle_Creation_Check) {
  std::vector<int> Obstacle_Extrema = Map.Build_Obstacle(Obstacle[0]);
  std::vector<int> Test_Obstacle = { 4, 0, 0, 6, 10, 10 };
  ASSERT_EQ(Obstacle_Extrema, Test_Obstacle);
}

/**
 * @brief Unit Test to Check whether the points in World are correctly
 * represented in Discretized workspace.
 */
TEST(Build_Map, Node_Creation_Check) {
  std::vector<double> Points = { 2, 2, 2 };
  std::vector<int> Test_Node = { 2, 2, 2 };
  std::vector<int> Built_Node = Map.Build_Node(Points);
  ASSERT_EQ(Built_Node, Test_Node);
}

/**
 * @brief Unit Test to Check whether the Points in Discretized Workspace are
 * correctly represented in the World.
 */
TEST(Build_Map, Coordinate_Generation_Check) {
  std::vector<int> Node_Init = { 2, 2, 2 };
  std::vector<double> Test_Coordinates = { 2, 2, 2 };
  std::vector<double> Generated_Coordinates = Map.Get_Coordinate(Node_Init);
  ASSERT_EQ(Test_Coordinates, Generated_Coordinates);
}

/**
 * @brief Unit Test to Check whether the Path Starts with the Goal Point.
 */
TEST(Planner, Goal_Point_Test) {
  auto coordinate = path[0];
  std::vector<int> Test_Node = { coordinate.x, coordinate.y, coordinate.z };
  std::vector<double> Coordinates = Map.Get_Coordinate(Test_Node);
  ASSERT_EQ(Coordinates[0], 2);
  ASSERT_EQ(Coordinates[1], 2);
  ASSERT_EQ(Coordinates[2], 2);
}

/**
 * @brief Unit Test to Check whether the Path ends with the Start Point.
 */
TEST(Planner, Start_Point_Test) {
  auto coordinate = path[2];
  std::vector<int> Test_Node = { coordinate.x, coordinate.y, coordinate.z };
  std::vector<double> Coordinates = Map.Get_Coordinate(Test_Node);
  ASSERT_EQ(Coordinates[0], 0);
  ASSERT_EQ(Coordinates[1], 0);
  ASSERT_EQ(Coordinates[2], 0);
}

/**
 * @brief Unit Test to Check Whether the Path is Planned correctly.
 */
TEST(Planner, Path_Generation) {
  int value = 2;
  for (auto& coordinate : path) {
    std::vector<int> Discrete_Node =
        { coordinate.x, coordinate.y, coordinate.z };
    std::vector<double> Coordinates = Map.Get_Coordinate(Discrete_Node);
    ASSERT_EQ(Coordinates[0], value);
    ASSERT_EQ(Coordinates[1], value);
    ASSERT_EQ(Coordinates[2], value);
    value = value - 1;
  }
}

/**
 * @brief Unit Test to Check Whether the Manhattan Distance Heuristic Works
 * as Intended.
 */
TEST(Planner, Manhattan_Test) {
  Plan.Set_Heuristic(Planner::Manhattan);
  auto path_1 = Plan.findPath({ Start_Node[0], Start_Node[1], Start_Node[2] },
                              { Goal_Node[0], Goal_Node[1], Goal_Node[2] });
  int value = 2;
  for (auto& coordinate : path_1) {
    std::vector<int> Discrete_Node =
        { coordinate.x, coordinate.y, coordinate.z };
    std::vector<double> Coordinates = Map.Get_Coordinate(Discrete_Node);
    ASSERT_EQ(Coordinates[0], value);
    ASSERT_EQ(Coordinates[1], value);
    ASSERT_EQ(Coordinates[2], value);
    value = value - 1;
  }
}

/**
 * @brief Unit Test to Check Whether the Planner Avoids Obstacles while Planning
 */

TEST(Planner, Collision_Test) {
  Plan.Add_Collision({ 1, 1, 1 });
  auto path_2 = Plan.findPath({ Start_Node[0], Start_Node[1], Start_Node[2] },
                              { Goal_Node[0], Goal_Node[1], Goal_Node[2] });
  for (auto& coordinate : path_2) {
    std::vector<int> Discrete_Node =
        { coordinate.x, coordinate.y, coordinate.z };
    std::vector<double> Coordinates = Map.Get_Coordinate(Discrete_Node);
    EXPECT_NE((Coordinates[0] * Coordinates[1] * Coordinates[2]), 1);
  }
}
