#include <iostream>
#include <cmath>
#include <vector>
#include "../include/Build_Map.h"

Build_Map::Build_Map(std::vector<double> _Boundary, double _xy_res,
                     double _z_res, double _margin) {
  Boundary = _Boundary;
  xy_res = _xy_res;
  z_res = _z_res;
  margin = _margin;
}

std::vector<int> Build_Map::World_Dimensions() {
  int World_X = (Boundary[3] - Boundary[0]) / xy_res;
  int World_Y = (Boundary[4] - Boundary[1]) / xy_res;
  int World_Z = (Boundary[5] - Boundary[2]) / z_res;
  std::vector<int> World = { World_X, World_Y, World_Z };
  return World;
}

// Obstacles

std::vector<int> Build_Map::Build_Obstacle(std::vector<double> v) {
  int minx = std::floor(((v[0] - Boundary[0] - margin) / xy_res));
  if (minx < 0) {
    minx = 0;
  }
  int miny = std::floor(((v[1] - Boundary[1] - margin) / xy_res));
  if (miny < 0) {
    miny = 0;
  }
  int minz = std::floor(((v[2] - Boundary[2] - margin) / z_res));
  if (minz < 0) {
    minz = 0;
  }

  int maxx = std::ceil(((v[3] - Boundary[0] + margin) / xy_res));
  if (maxx > World_X) {
    maxx = World_X;
  }
  int maxy = std::ceil(((v[4] - Boundary[1] + margin) / xy_res));
  if (maxy > World_Y) {
    maxy = World_Y;
  }
  int maxz = std::ceil(((v[5] - Boundary[2] + margin) / z_res));
  if (maxz > World_Z) {
    maxz = World_Z;
  }
  std::vector<int> Obstacle_Extrema = { minx, miny, minz, maxx, maxy, maxz };
  return Obstacle_Extrema;
}

std::vector<int> Build_Map::Build_Node(std::vector<double> Discrete) {
  int X_Init = (Discrete[0] - Boundary[0]) / xy_res;
  int Y_Init = (Discrete[1] - Boundary[1]) / xy_res;
  int Z_Init = (Discrete[2] - Boundary[2]) / z_res;
  std::vector<int> Built_Node = { X_Init, Y_Init, Z_Init };
  return Built_Node;
}

std::vector<double> Build_Map::Get_Coordinate(std::vector<int> Node_Init) {
  double X_Coordinate = Boundary[0] + (Node_Init[0] * xy_res);
  double Y_Coordinate = Boundary[1] + (Node_Init[1] * xy_res);
  double Z_Coordinate = Boundary[2] + (Node_Init[2] * z_res);
  std::vector<double> Coordinates_Generated = { X_Coordinate, Y_Coordinate,
      Z_Coordinate };
  return Coordinates_Generated;
}

Build_Map::~Build_Map() {
}
