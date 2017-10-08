#ifndef MID_TERM_A_STAR_3D_INCLUDE_PLANNER_H_
#define MID_TERM_A_STAR_3D_INCLUDE_PLANNER_H_

#include <vector>
#include <functional>
#include <set>

struct Vec3i {
  int x, y, z;
  bool operator ==(const Vec3i& coordinates_);
};

struct Node {
  double G, H;
  Vec3i coordinates;
  Node *Parent;
  explicit Node(Vec3i coord_, Node *Parent_ = nullptr);
  double Get_Score();
};

class Planner {
  bool Detect_Collision(Vec3i coordinates_);
  Node* Find_Node(std::set<Node*>& nodes_, Vec3i coordinates_);

 public:
  Planner();
  void Set_World_Size(Vec3i World_Size_);
  void Set_Heuristic(std::function<double(Vec3i, Vec3i)> heuristic_);
  std::vector<Vec3i> findPath(Vec3i Current_, Vec3i Neighbor_);
  void Add_Collision(Vec3i coordinates_);
  std::function<double(Vec3i, Vec3i)> heuristic;
  std::vector<Vec3i> direction, walls;
  Vec3i World_Size;
  double Directions;
  virtual ~Planner();
};

class Heuristic {
  static Vec3i Distance(Vec3i Current_, Vec3i Neighbor_);

 public:
  static double Euclidean(Vec3i Current_, Vec3i Neighbor_);
};

#endif  // MID_TERM_A_STAR_3D_INCLUDE_PLANNER_H_
