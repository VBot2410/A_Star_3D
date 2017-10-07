#include "Planner.h"
#include <algorithm>

Node::Node(Vec3i coordinates_, Node *Parent_) {
  Parent = Parent_;
  coordinates = coordinates_;
  G = H = 0;
}

double Node::Get_Score() {
  return G + H;
}

bool Vec3i::operator ==(const Vec3i& coordinates_) {
  return (x == coordinates_.x && y == coordinates_.y && z == coordinates_.z);
}

Vec3i operator +(const Vec3i& left_, const Vec3i& right_) {
  return {left_.x + right_.x, left_.y + right_.y,left_.z + right_.z};
}

Planner::Planner() {
  Set_Heuristic(&Heuristic::Euclidean);
  direction = {
    { 0,0,1}, {0,1,0}, {1,0,0}, {0,0,-1}, {0,-1,0}, {-1,0,0}, {0,1,1}, {1,0,1}, {1,1,0},
    { 0,-1,-1}, {-1,0,-1}, {-1,-1,0},
    { 0,1,-1}, {0,-1,1}, {1,0,-1}, {-1,0,1}, {1,-1,0}, {-1,1,0}, {1,1,-1}, {1,-1,1},
    { -1,1,1}, {1,-1,-1}, {-1,-1,1}, {-1,1,-1}, {1,1,1}, {-1,-1,-1}
  };
}

void Planner::Set_World_Size(Vec3i World_Size_) {
  World_Size = World_Size_;
}

void Planner::Set_Heuristic(std::function<double(Vec3i, Vec3i)> heuristic_) {
  heuristic = std::bind(heuristic_, std::placeholders::_1,
                        std::placeholders::_2);
}

void Planner::Add_Collision(Vec3i coordinates_) {
  walls.push_back(coordinates_);
}

std::vector<Vec3i> Planner::findPath(Vec3i Now_, Vec3i Neighbor_) {
  Node *current = nullptr;
  std::set<Node*> Open_Set, Closed_Set;
  Open_Set.insert(new Node(Now_));

  while (!Open_Set.empty()) {
    current = *Open_Set.begin();
    for (auto node : Open_Set) {
      if (node->Get_Score() <= current->Get_Score()) {
        current = node;
      }
    }

    if (current->coordinates == Neighbor_) {
      break;
    }

    Closed_Set.insert(current);
    Open_Set.erase(std::find(Open_Set.begin(), Open_Set.end(), current));

    for (double i = 0; i < 26; ++i) {
      Vec3i newCoordinates(current->coordinates + direction[i]);
      if (Detect_Collision(newCoordinates)
          || Find_Node(Closed_Set, newCoordinates)) {
        continue;
      }

      double Total_Cost = current->G
          + ((i < 6) ? 100 : ((i > 5 && i < 18) ? 141 : 173));

      Node *successor = Find_Node(Open_Set, newCoordinates);
      if (successor == nullptr) {
        successor = new Node(newCoordinates, current);
        successor->G = Total_Cost;
        successor->H = heuristic(successor->coordinates, Neighbor_);
        Open_Set.insert(successor);
      } else if (Total_Cost < successor->G) {
        successor->Parent = current;
        successor->G = Total_Cost;
      }
    }
  }

  std::vector<Vec3i> path;
  while (current != nullptr) {
    path.push_back(current->coordinates);
    current = current->Parent;
  }

  return path;
}

Node* Planner::Find_Node(std::set<Node*>& nodes_, Vec3i coordinates_) {
  for (auto node : nodes_) {
    if (node->coordinates == coordinates_) {
      return node;
    }
  }
  return nullptr;
}

bool Planner::Detect_Collision(Vec3i coordinates_) {
  if (coordinates_.x < 0 || coordinates_.x >= World_Size.x || coordinates_.y < 0
      || coordinates_.y >= World_Size.y || coordinates_.z < 0
      || coordinates_.z >= World_Size.z
      || std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {
    return true;
  }
  return false;
}

Vec3i Heuristic::Distance(Vec3i Now_, Vec3i Neighbor_) {
  return {abs(Now_.x - Neighbor_.x), abs(Now_.y - Neighbor_.y),abs(Now_.z - Neighbor_.z)};
}

double Heuristic::Euclidean(Vec3i Now_, Vec3i Neighbor_) {
  auto delta = std::move(Distance(Now_, Neighbor_));
  return static_cast<double>(100
      * sqrt(pow(delta.x, 2) + pow(delta.y, 2) + pow(delta.z, 2)));
}

