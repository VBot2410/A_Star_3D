/**
 * @file Planner.h
 * @brief This file contains the variables and function declarations for
 * Planner Class which takes the Environment data built by the Build_Map
 * Class and Uses A* to plan the Shortest Path while avoiding all Obstacles.
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

#ifndef MID_TERM_A_STAR_3D_INCLUDE_PLANNER_H_
#define MID_TERM_A_STAR_3D_INCLUDE_PLANNER_H_

/* --Includes-- */
#include <vector>
#include <functional>
#include <set>

/**
 * @brief Vec3i of type Struct which Builds points with x,y,z values
 */
struct Vec3i {
  /** x,y,z of type integer */
  int x, y, z;
  /** operator of type boolean */
  bool operator ==(const Vec3i& coordinates_);
};

/**
 * @brief Node of type Struct which Stores Various Property values of the Nodes
 */
struct Node {
  /**
   * G of type double which stores the Cost-to-Start value
   * H of type double which is the heuristic value
   */
  double G, H;
  /** coordinates of type Vec3i struct which stores coordinates of Node */
  Vec3i coordinates;
  /** Parent of type pointer which points to the parent node of current node */
  Node *Parent;
  /** Constructor for Node Struct */
  explicit Node(Vec3i, Node *Parent_ = nullptr);
  /** Get_Score returns the F value of type double (F=G+H) */
  double Get_Score();
};

/**
 * @brief Declaration of Class Planner
 */
class Planner {
  /** Private Access Specifier */

  /**
   * Detect_Collision function of return type boolean returns true if the
   * Node is inside an obstacle.
   */
  bool Detect_Collision(Vec3i);
  /** Find Node finds the Location of Given Node in List of Nodes */
  Node* Find_Node(std::set<Node*>& nodes_, Vec3i);
  /** Distance finds the Distance between two nodes */
  static Vec3i Distance(Vec3i, Vec3i);

 public:  ///< Public Access Specifier
  explicit Planner(Vec3i);  ///< Constructor for Class Planner
  static double Euclidean(Vec3i, Vec3i);  ///< Euclidean Distance Heuristic
  static double Manhattan(Vec3i, Vec3i);  ///< Manhattan Distance Heuristic
  /** Set_Heuristic sets the Heuristic Function */
  void Set_Heuristic(std::function<double(Vec3i, Vec3i)>);
  /** findPath Plans the Path from Start to Goal Point */
  std::vector<Vec3i> findPath(Vec3i, Vec3i);
  /** Add_Collision adds the Nodes to Obstacle List */
  void Add_Collision(Vec3i);
  /** Heuristic Function */
  std::function<double(Vec3i, Vec3i)> heuristic;
  /**
   * direction contains direction of movement from current to neighbor node.
   * walls contains all Obstacle Nodes
   */
  std::vector<Vec3i> direction, walls;
  /** World_Size of return type Vec3i struct contains world dimensions */
  Vec3i World_Size;
  virtual ~Planner();  ///< Destructor for Planner Class
};

#endif  // MID_TERM_A_STAR_3D_INCLUDE_PLANNER_H_
