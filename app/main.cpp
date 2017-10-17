#include <iostream>
#include <vector>
#include <cmath>
#include "../include/Planner.h"
#include "../include/Build_Map.h"

int main() {
  double xy_res = 0.25;
  double z_res = 0.25;
  double margin = 0.2;
  std::vector<double> Boundary = { 0.0, -5.0, 0.0, 10.0, 20.0, 6.0 };
  std::vector<std::vector<double>> Obstacle = {
      { 0.0, 2.0, 0.0, 10.0, 2.5, 1.5 }, { 0.0, 2.0, 4.5, 10.0, 2.5, 6.0 }, {
          0.0, 2.0, 1.5, 3.0, 2.5, 4.5 } };

  // Map_Build

  Build_Map Map = Build_Map(Boundary, xy_res, z_res, margin);

  std::vector<int> World = Map.World_Dimensions();
  Planner Plan = Planner( { World[0], World[1], World[2] });

  for (const std::vector<double> &v : Obstacle) {
    std::vector<int> Obstacle_Extrema = Map.Build_Obstacle(v);
    for (int Counter_X = Obstacle_Extrema[0]; Counter_X != Obstacle_Extrema[3];
        Counter_X++) {
      for (int Counter_Y = Obstacle_Extrema[1];
          Counter_Y != Obstacle_Extrema[4]; Counter_Y++) {
        for (int Counter_Z = Obstacle_Extrema[2];
            Counter_Z != Obstacle_Extrema[5]; Counter_Z++) {
          Plan.Add_Collision( { Counter_X, Counter_Y, Counter_Z });
        }
      }
    }
  }

// Calculations
  Plan.Set_Heuristic(Planner::Manhattan);
  std::cout << "Calculating Shortest Path ... \n";
  std::vector<double> Start = { 0, 0.5, 3 };
  std::vector<double> Goal = { 3.9, 6.4, 0 };
  if ((Start[0] < Boundary[0] || Start[0] > Boundary[3])
      || (Start[1] < Boundary[1] || Start[1] > Boundary[4])
      || (Start[2] < Boundary[2] || Start[2] > Boundary[5])) {
    std::cout << "Start Point Lies Out of Workspace.";
  } else if ((Goal[0] < Boundary[0] || Goal[0] > Boundary[3])
      || (Goal[1] < Boundary[1] || Goal[1] > Boundary[4])
      || (Goal[2] < Boundary[2] || Goal[2] > Boundary[5])) {
    std::cout << "Goal Point Lies Out of Workspace.";
  } else {
    std::vector<int> Start_Node = Map.Build_Node(Start);
    std::vector<int> Goal_Node = Map.Build_Node(Goal);
    auto path = Plan.findPath( { Start_Node[0], Start_Node[1], Start_Node[2] },
                              { Goal_Node[0], Goal_Node[1], Goal_Node[2] });
    for (auto& coordinate : path) {
      std::vector<int> Discrete_Node = { coordinate.x, coordinate.y, coordinate
          .z };
      std::vector<double> Coordinates = Map.Get_Coordinate(Discrete_Node);
      std::cout << Coordinates[0] << " " << Coordinates[1] << " "
          << Coordinates[2] << "\n";
    }
  }
  return 0;
}
