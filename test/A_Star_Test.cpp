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
std::vector<int> World = Map.World_Dimensions();
Planner Plan = Planner({ World[0], World[1], World[2] });
std::vector<double> Start = { 0, 0, 0 };
std::vector<double> Goal = { 2, 2, 2 };
std::vector<int> Start_Node = Map.Build_Node(Start);
std::vector<int> Goal_Node = Map.Build_Node(Goal);
auto path = Plan.findPath({ Start_Node[0], Start_Node[1], Start_Node[2] }, {
                              Goal_Node[0], Goal_Node[1], Goal_Node[2] });

TEST(Build_Map, World_Dimensions_Check) {
  std::vector<int> World = Map.World_Dimensions();
  std::vector<int> Test_World = { 10, 10, 10 };
  ASSERT_EQ(World, Test_World);
}

TEST(Build_Map, Obstacle_Creation_Check) {
  std::vector<int> Obstacle_Extrema = Map.Build_Obstacle(Obstacle[0]);
  std::vector<int> Test_Obstacle = { 4, 0, 0, 6, 10, 10 };
  ASSERT_EQ(Obstacle_Extrema, Test_Obstacle);
}

TEST(Build_Map, Node_Creation_Check) {
  std::vector<double> Points = { 2, 2, 2 };
  std::vector<int> Test_Node = { 2, 2, 2 };
  std::vector<int> Built_Node = Map.Build_Node(Points);
  ASSERT_EQ(Built_Node, Test_Node);
}

TEST(Build_Map, Coordinate_Generation_Check) {
  std::vector<int> Node_Init = { 2, 2, 2 };
  std::vector<double> Test_Coordinates = { 2, 2, 2 };
  std::vector<double> Generated_Coordinates = Map.Get_Coordinate(Node_Init);
  ASSERT_EQ(Test_Coordinates, Generated_Coordinates);
}

TEST(Planner, Goal_Point_Test) {
  auto coordinate = path[0];
  std::vector<int> Test_Node = { coordinate.x, coordinate.y, coordinate.z };
  std::vector<double> Coordinates = Map.Get_Coordinate(Test_Node);
  ASSERT_EQ(Coordinates[0], 2);
  ASSERT_EQ(Coordinates[1], 2);
  ASSERT_EQ(Coordinates[2], 2);
}

TEST(Planner, Start_Point_Test) {
  auto coordinate = path[2];
  std::vector<int> Test_Node = { coordinate.x, coordinate.y, coordinate.z };
  std::vector<double> Coordinates = Map.Get_Coordinate(Test_Node);
  ASSERT_EQ(Coordinates[0], 0);
  ASSERT_EQ(Coordinates[1], 0);
  ASSERT_EQ(Coordinates[2], 0);
}

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
