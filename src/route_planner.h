#pragma once

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"


class RoutePlanner {
  public:
    //constructor
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);
    //public 'getter' function to get the distance
    float GetDistance() {return distance;}
    //AStarSearch main method
    void AStarSearch();

  private:
    //m_Model will refer to the model that you will be performing A* search on
    RouteModel &m_Model;
    //These will point to the nodes in the model which are closest to our starting and ending points.
    RouteModel::Node *start_node;
    RouteModel::Node *end_node;
    
    float distance;
    vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *current_node);
    float CalculateHValue(RouteModel::Node const *current_node);
    vector<RouteModel::Node*> open_list;
    RouteModel::Node *NextNode();
    void AddNeighbors(RouteModel::Node *current_node);
};
