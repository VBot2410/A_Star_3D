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
  explicit Node(Vec3i, Node *Parent_ = nullptr);
  double Get_Score();
};

class Planner {
  bool Detect_Collision(Vec3i);
  Node* Find_Node(std::set<Node*>& nodes_, Vec3i);
  static Vec3i Distance(Vec3i, Vec3i);

 public:
  explicit Planner(Vec3i);
  static double Euclidean(Vec3i, Vec3i);
  void Set_Heuristic(std::function<double(Vec3i, Vec3i)>);
  std::vector<Vec3i> findPath(Vec3i, Vec3i);
  void Add_Collision(Vec3i);
  std::function<double(Vec3i, Vec3i)> heuristic;
  std::vector<Vec3i> direction, walls;
  Vec3i World_Size;
  virtual ~Planner();
};

#endif  // MID_TERM_A_STAR_3D_INCLUDE_PLANNER_H_
