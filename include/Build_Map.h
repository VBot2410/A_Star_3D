#ifndef INCLUDE_BUILD_MAP_H_
#define INCLUDE_BUILD_MAP_H_

//#include <vector>
#include <functional>
#include <set>
#include "Planner.h"


class Build_Map {
 public:
  Build_Map(std::vector<double>,double,double,double);
  double x0,y0,z0,x1,y1,z1;
  std::vector<double>Boundary;
  double xy_res,z_res,margin;
  int World_X, World_Y, World_Z;
  std::vector<int>World_Dimensions();
  std::vector<int>Build_Obstacle(std::vector<double>);
  std::vector<int>Build_Node(std::vector<double>);
  std::vector<double>Get_Coordinate(std::vector<int>);
  virtual ~Build_Map();
};


#endif /* INCLUDE_BUILD_MAP_H_ */
