#include <iostream>
#include <cmath>
#include "Build_Map.h"

Build_Map::Build_Map(std::vector<double>_Boundary,double _xy_res,double _z_res,double _margin){
  Boundary=_Boundary;
  xy_res=_xy_res;
  z_res=_z_res;
  margin=_margin;
}


std::vector<int> Build_Map::World_Dimensions(){
  return std::vector<int> World={99,99,99};
}

std::vector<int>Build_Map::Build_Obstacle(std::vector<double>v){
    return std::vector<int>Obstacle_Extrema={0,0,0,1,1,1};
}

std::vector<int>Build_Map::Build_Node(std::vector<double>Discrete){
  return std::vector<int>Built_Node={99,99,99};;
}

std::vector<double>Build_Map::Get_Coordinate(std::vector<int>Node_Init){
  return   std::vector<double>Coordinates_Generated={99,99,99};
}

    Build_Map::~Build_Map() {
    }
