#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"


class RoutePlanner {
  public:
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);
    // Add public variables or methods declarations here.
    float GetDistance() const { return distance;}
    void AStarSearch();

  private:
    // Add private variables or methods declarations here.
    RouteModel &m_Model;
    RouteModel::Node *start_node, *end_node;
    float distance;
    std::vector<RouteModel::Node> ConstructFinalPath (RouteModel::Node *current_node);
    //`CalculateHValue` should accept a `const` pointer to a `RouteModel::Node` object, and it should return a `float`.
    float CalculateHValue (const RouteModel::Node *);
    std::vector<RouteModel::Node*> open_list;
    RouteModel::Node *NextNode();
    void AddNeighbors(RouteModel::Node *current_node);
};

#endif