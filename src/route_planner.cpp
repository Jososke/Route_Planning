#include "route_planner.h"
#include <algorithm>

//The constructor takes one RouteModel reference and four floats as arguments. 
//The RouteModel reference is assigned to the m_Model variable. 
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

//full AStarSearch Routine
void RoutePlanner::AStarSearch() {
    //Set start_node->visited to be true.
    start_node->visited = true;
    //Push start_node to the back of open_list.
    open_list.push_back(start_node);
    //Create a pointer RouteModel::Node *current_node and initialize the pointer to nullptr.
    RouteModel::Node *current_node = nullptr;
    //while the open_list size is greater than 0:
    while (open_list.size() > 0) {
        //Set the current_node pointer to the results of calling NextNode.
        current_node = NextNode();
        //if the distance from current_node to the end_node is 0:
        if (current_node->distance(*end_node) == 0) {
            //Call ConstructFinalPath using current_node and set m_Model.path with the results.
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        //else call AddNeighbors with the current_node.
        else {
            AddNeighbors(current_node);
        }
    }
}

//ConstructFinalPath that takes a RouteModel::Node pointer and iteratively moves the 
//sequence of parents, storing each node in the sequence, until the starting node is reached.
vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    vector<RouteModel::Node> path_found;
    while (current_node->parent != nullptr) {
        path_found.push_back(*current_node);
        distance += current_node->distance(*(current_node->parent));
        current_node = current_node->parent;
    }
    distance *= m_Model.MetricScale();
    path_found.push_back(*current_node);
    return path_found;
}

//CalculateHValue, which calculates the h-value for a given node. In this project, 
//h-value will be computed as the euclidean distance from the node to the end node. 
float RoutePlanner::CalculateHValue(RouteModel::Node const *current_node) {
    return current_node->distance(*end_node);
}

//RoutePlanner::NextNode method which will sort the list of open nodes in the A* search, 
//return the node with the lowest f-value, and remove the node from the list of open nodes.
RouteModel::Node *RoutePlanner::NextNode() {
    //using lambda expression sorting method
    sort(open_list.begin(), open_list.end(), [](const auto &node1, const auto &node2) {
        return node1->h_value + node1->g_value < node2->h_value + node2->g_value;
    });
    //using front method to return the reference to the first member of the list
    RouteModel::Node *f_lowest = open_list.front();
    //using begin to reference the iterator (which is more like a pointer)
    open_list.erase(open_list.begin());
    //returning the lowest value
    return f_lowest;
}

//This method will take each neighbor of the current node in the A* search, 
//set the neighbor's g-value, h-value, and parent, and add the neighbor to the open list. 
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto neighbor : current_node->neighbors) {
        neighbor->parent = current_node;
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        neighbor->h_value = CalculateHValue(neighbor);
        open_list.push_back(neighbor);
        neighbor->visited = true;
    }
}

