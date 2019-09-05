#pragma once

#include "model.h"

#include <limits>
#include <cmath>
#include <unordered_map>
#include <iostream>
#include <math.h>

using std::vector;

class RouteModel : public Model {

  public:
    class Node : public Model::Node {
      public:
        //The Model::Node class that exists in the current code doesn't contain 
        //all the data that would be needed to perfom an A* search. In order to perform a search, 
        //it would be ideal for each node to contain at least the following information:
        Node * parent = nullptr;
        float h_value = std::numeric_limits<float>::max(); 
        float g_value = 0.0;
        bool visited = false;
        vector<Node *> neighbors;

        //Return the euclidean distance from the current node to the node passed in.
        float distance(Model::Node passed_node) const {
            return sqrt(pow(x - passed_node.x, 2) + pow(y - passed_node.y, 2));
        }

        void FindNeighbors();

        Node(){}
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}

      private:
        // Add private Node variables and methods here.
        int index;
        RouteModel * parent_model = nullptr;
        Node * FindNeighbor(vector<int> node_indices);
    };

    //public 'getter' method    
    auto &SNodes() { return m_Nodes; }
    RouteModel(const vector<std::byte> &xml);  
    vector<Node> path; // This variable will eventually store the path that is found by the A* search.
    //public 'getter' function to return a reference for testing primarily
    auto &GetNodeToRoadMap() { return node_to_road; }
    Node &FindClosestNode(float x, float y);

  private:
    //m_Nodes will store all of the nodes from the Open Street Map data
    vector<Node> m_Nodes = {};

    //Initialization of hash table of Node index values to a vector of 
    //Road pointers that those nodes belong to.
    std::unordered_map<int, vector<const Model::Road*>> node_to_road;
    void CreateNodeToRoadHashmap();
};
