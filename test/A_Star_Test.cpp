/*
 * A_Star_Test.cpp
 *
 *  Created on: Oct 6, 2017
 *      Author: viki
 */

#include <gtest/gtest.h>
#include <vector>
#include "../include/Build_Map.h"
#include "../include/Planner.h"

double xy_res = 1;
double z_res = 1;
double margin = 0.1;
std::vector<double> Boundary = { 0.0, 0.0, 0.0, 10.0, 10.0, 10.0 };
std::vector<std::vector<double>> Obstacle = { { 5.0, 0.0, 0.0, 5.0, 10, 10 } };
Build_Map Map = Build_Map(Boundary, xy_res, z_res, margin);

TEST(Build_Map, World_Dimensions_Check) {
  std::vector<int> World = Map.World_Dimensions();
  std::vector<int> Test_World = { 10, 10, 10 };
  ASSERT_EQ(World, Test_World);
}

TEST(Build_Map,Obstacle_Creation_Check) {
  Map.World_X = 10;
  Map.World_Y = 10;
  Map.World_Z = 10;
  std::vector<int> Obstacle_Extrema = Map.Build_Obstacle(Obstacle[0]);
  std::vector<int> Test_Obstacle = { 4, 0, 0, 6, 10, 10 };
  ASSERT_EQ(Obstacle_Extrema, Test_Obstacle);
}

TEST(Build_Map,Node_Creation_Check) {
  std::vector<double> Points = { 2, 2, 2 };
  std::vector<int> Test_Node = { 2, 2, 2 };
  std::vector<int> Built_Node = Map.Build_Node(Points);
  ASSERT_EQ(Built_Node, Test_Node);
}

TEST(Build_Map,Coordinate_Generation_Check) {
  std::vector<int> Node_Init = { 2, 2, 2 };
  std::vector<double> Test_Coordinates = { 2, 2, 2 };
  std::vector<double> Generated_Coordinates = Map.Get_Coordinate(Node_Init);
  ASSERT_EQ(Test_Coordinates, Generated_Coordinates);
}
