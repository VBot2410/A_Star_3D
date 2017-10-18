/**
 * @file Planner.cpp
 * @brief This file contains the code for Planner Class which takes the
 * Environment data built by the Build_Map Class and Uses A* to plan the
 * Shortest Path while avoiding all Obstacles.
 *
 * @author Vaibhav Bhilare
 * @copyright 2017, Vaibhav Bhilare
 *
 * MIT License
 * Copyright (c) 2017 Vaibhav Bhilare
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* --Includes-- */
#include <iostream>
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>
#include <set>
#include "../include/Planner.h"

/**
 * @brief Constructor for Node Struct
 * Initializes the Node features to default values
 * @param coordinates of type Vec3i struct stores Coordinates of the Node
 * @param Parent of type Pointer stores pointer to the Parent of current Node
 * @param G Cost-to-Start of type double initialized to 0
 * @param H Heuristic of type double initialized to 0
 */
Node::Node(Vec3i coordinates_, Node *Parent_)
    : coordinates(coordinates_) {
  Parent = Parent_;
  G = H = 0;
}

/**
 * @brief Get_Score function of return type double from Node struct
 * It Adds the G and H values of current node to give the F value
 * @return (G+H) of type double
 */
double Node::Get_Score() {
  return G + H;
}

/**
 * @brief operator of return type boolean from Vec3i struct
 * It compares the X,Y,Z, values of given coordinate with reference
 * @return boolean true if coordinate matches the reference else return false
 */
bool Vec3i::operator ==(const Vec3i& coordinates_) {
  return (x == coordinates_.x && y == coordinates_.y && z == coordinates_.z);
}

/**
 * @brief Addition Operator of return type Vec3i struct
 * It adds corresonding X,Y,Z values of two Vec3i structs and returns a
 * new Vec3i Struct
 * @return Vec3i type Struct
 */
Vec3i operator +(const Vec3i& left_, const Vec3i& right_) {
  return {left_.x + right_.x, left_.y + right_.y, left_.z + right_.z};
}

/**
 * @brief Constructor for class Planner
 * It initializes the Planner with World Dimensions, Default Heuristic,
 * Set of possible Directions
 * @param direction of type double initializes set of possible Directions
 * @param World_Size of size Vec3i which stores the World Size
 */
Planner::Planner(Vec3i World_Size_) {
  Set_Heuristic(&Planner::Euclidean);  ///< Set default heuristic to Euclidean
  direction = {
    { 0 , 0 , 1}, {0 , 1, 0}, {1, 0, 0}, {0, 0, -1},
    { 0, -1, 0}, {-1, 0, 0}, {0 , 1, 1}, {1, 0, 1},
    { 1, 1, 0}, {0, -1, -1}, {-1, 0, -1}, {-1, -1, 0},
    { 0, 1, -1}, {0, -1, 1}, {1, 0, -1}, {-1, 0, 1},
    { 1, -1, 0}, {-1, 1, 0}, {1, 1, -1}, {1, -1, 1},
    { -1, 1, 1}, {1, -1, -1}, {-1, -1, 1}, {-1, 1, -1},
    { 1, 1, 1}, {-1, -1, -1}
  };
  World_Size = World_Size_;
}

/**
 * @brief Set_Heuristic sets the Heuristic Behavior of neighbor Search
 * @param heuristic builds heuristic given a heuristic function input
 * @return void
 */
void Planner::Set_Heuristic(std::function<double(Vec3i, Vec3i)> heuristic_) {
  heuristic = std::bind(heuristic_, std::placeholders::_1,
                        std::placeholders::_2);
}

/**
 * @brief Add_Collision adds the Collision point to the walls list
 * @cordinates_ of type Vec3i contains the coordinates list of Obstacle
 * @return void
 */
void Planner::Add_Collision(Vec3i coordinates_) {
  walls.push_back(coordinates_);
}

/**
 * @brief findPath Finds the path from Start to Goal Point
 * @param Start_ of type Vec3i struct which stores Start point coordinates
 * @param Goal_ of type Vec3i struct which store Goal point coordinates
 * @return vector of Vec3i type which contains Path from start to goal
 */
std::vector<Vec3i> Planner::findPath(Vec3i Start_, Vec3i Goal_) {
  Node *current = nullptr;  ///< Set Current Node pointer as Null pointer
  std::set<Node*> Open_Set, Closed_Set;  ///< Initialize the Open & Closed Sets
  Open_Set.insert(new Node(Start_));  ///< Insert Start node to Open Set

  while (true) {
    /** Set current node pointer to First node of Open Set */
    current = *Open_Set.begin();

    /** Search for the node with least F value and set it as Current Node */
    for (auto node : Open_Set) {
      if (node->Get_Score() <= current->Get_Score()) {
        current = node;
      }
    }
    /** If Current Node is Goal, Then Stop Searching */
    if (current->coordinates == Goal_) {
      break;
    }

    /** Insert Current Node to the Closed Set */
    Closed_Set.insert(current);

    /** Remove Current Node from Open Set*/
    Open_Set.erase(std::find(Open_Set.begin(), Open_Set.end(), current));
    /** From all movable directions, check the neighbors*/
    for (double i = 0; i < 26; ++i) {
      Vec3i newCoordinates(current->coordinates + direction[i]);
      /** Check if Collision Happens */
      if (Detect_Collision(newCoordinates)
          || Find_Node(Closed_Set, newCoordinates)) {
        continue;
      }
      /** Find F value of Neighbor */
      double Total_Cost = current->G
          + ((i < 6) ? 100 : ((i > 5 && i < 18) ? 141 : 173));
      Node *successor = Find_Node(Open_Set, newCoordinates);
      if (successor == nullptr) {
        successor = new Node(newCoordinates, current);
        successor->G = Total_Cost;
        successor->H = heuristic(successor->coordinates, Goal_);
        Open_Set.insert(successor);
      } else if (Total_Cost < successor->G) {
        /** Set Parent Node to Successor Node */
        successor->Parent = current;
        /** Set G value of Successor */
        successor->G = Total_Cost;
      }
    }
    /** Print Path Not Found if Open List is Empty */
    if (Open_Set.empty()) {
      std::cout << "Path Not Found";
      break;
    }
  }
/** Store Path from Start to Goal in path vector */
  std::vector<Vec3i> path;
  while (current != nullptr) {
    path.push_back(current->coordinates);
    current = current->Parent;
  }
  return path;  ///< Return Calculated path
}

/**
 * @brief Find_Node finds the node representing given coordinates in Nodes list
 * @param nodes_ Nodes list
 * @param coordinates Vec3i struct of Coordinate values
 * @return pointer to the node if found else return null pointer
 */
Node* Planner::Find_Node(std::set<Node*>& nodes_, Vec3i coordinates_) {
  for (auto node : nodes_) {
    if (node->coordinates == coordinates_) {
      return node;  ///< Return pointer to the node if Node Found
    }
  }
  return nullptr;  ///< Return null pointer if node not found
}

/**
 * @brief Detect_Collision checks if the point lies inside the obstacle
 * @param coordinates_ has a type Vec3i and stores the coordinates
 * @return true if point lies inside the obstacle else false
 */
bool Planner::Detect_Collision(Vec3i coordinates_) {
  if (coordinates_.x < 0 || coordinates_.x >= World_Size.x || coordinates_.y < 0
      || coordinates_.y >= World_Size.y || coordinates_.z < 0
      || coordinates_.z >= World_Size.z
      || std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {
    return true;  ///< return true of collision happens
  }
  return false;  ///< return false if collision doesn't happen
}

/**
 * @brief Distance gives the distance between two points
 * @param Now_ has type Vec3i struct
 * @param Neighbor_ has type Vec3i struct
 * @return Vec3i struct type Absolute Distance between two points
 */
Vec3i Planner::Distance(Vec3i Now_, Vec3i Neighbor_) {
  return {abs(Now_.x - Neighbor_.x), abs(Now_.y - Neighbor_.y),
    abs(Now_.z - Neighbor_.z)};
}

/**
 * @brief Euclidean is a Euclidean Distance Heuristic function
 * @param Now_ has type Vec3i struct
 * @param Neighbor_ has type Vec3i struct
 * @return double type Euclidean Distance between two points
 */
double Planner::Euclidean(Vec3i Now_, Vec3i Neighbor_) {
  auto delta = std::move(Distance(Now_, Neighbor_));
  return static_cast<double>(100
      * sqrt(pow(delta.x, 2) + pow(delta.y, 2) + pow(delta.z, 2)));
}

/**
 * @brief Manhattan is a Manhattan Distance Heuristic function
 * @param Now_ has type Vec3i struct
 * @param Neighbor_ has type Vec3i struct
 * @return double type Manhattan Distance between two points
 */
double Planner::Manhattan(Vec3i Now_, Vec3i Neighbor_) {
  auto delta = std::move(Distance(Now_, Neighbor_));
  return static_cast<double>(100 * (delta.x + delta.y + delta.z));
}

/** Destructor for Planner Class */
Planner::~Planner() {
}
