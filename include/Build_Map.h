#ifndef INCLUDE_BUILD_MAP_H_
#define INCLUDE_BUILD_MAP_H_

#include <functional>
#include <set>
#include "Planner.h"


class Build_Map {
 public:
  Build_Map(std::vector<double>,double,double,double);
  std::vector<int>World_Dimensions();
  std::vector<int>Build_Obstacle(std::vector<double>);
  std::vector<int>Build_Node(std::vector<double>);
  std::vector<double>Get_Coordinate(std::vector<int>);
  virtual ~Build_Map();
};


#endif /* INCLUDE_BUILD_MAP_H_ */
