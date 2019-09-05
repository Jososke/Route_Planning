#include "route_model.h"
#include <iostream>

using std::vector;
using std::byte;

//When the RouteModel constructor is called, it calls the Model constructor 
//with the open street map data. When this happens, a collection of Model:Node objects are created. 
//However, in order to perform the A* search, you will need to use RouteModel::Node objects instead.
RouteModel::RouteModel(const vector<byte> &xml) : Model(xml) {
    int counter = 0;
    //For each Model node in the loop, use the RouteModel::Node constructor 
    //to create a new node, and push the new node to the back of m_Nodes.
    for(Model::Node new_node : this->Nodes()) {
        m_Nodes.push_back(Node(counter, this, new_node));
        counter++;
    }
    CreateNodeToRoadHashmap();
}

//Function of hash table of Node index values to a vector of 
//Road pointers that those nodes belong to.
void RouteModel::CreateNodeToRoadHashmap() {
    //iterates through the vector given by calling Roads()
    for(auto &road: Roads()) {
        //checking that the type is not a footway
        if (road.type != Model::Road::Type::Footway) {
            //looping over each node_idx in the way
            for (auto node_idx : Ways()[road.way].nodes) {
                //if the node index is not in the hashmap yet, set the value for the node_idx key
                //to be an empty vector of const Model::Road * objects
                if (node_to_road.find(node_idx) == node_to_road.end()) {
                    node_to_road[node_idx] = vector<const Model::Road *> {};
                }
                //Push a pointer to the current road in the loop to the back of the vector given 
                //by the node_idx key in node_to_road.
                node_to_road[node_idx].push_back(&road);
            }
        }
    }
}

//The goal of FindNeighbor is to return a pointer to the closest unvisited node from a 
//vector of node indices, where the distance is measured to the current node (this). 
RouteModel::Node *RouteModel::Node::FindNeighbor(vector<int> node_indices) {
    Node *closest_node = nullptr;
    Node node;
    //loop through the node_indices vector to find the closest unvisited node. 
    for (int node_index : node_indices) {
        node = parent_model->SNodes()[node_index];
        if (this->distance(node) != 0 && !node.visited) {
            if (closest_node == nullptr || (this->distance(node) < this->distance(*closest_node))) {
                closest_node = &parent_model->SNodes()[node_index];
            }
        }
    }
    return closest_node;
}

//The goal of FindNeighbors is to populate the neighbors vector of the 
//current Node object (the vector this->neighbors).
void RouteModel::Node::FindNeighbors() {
    for (auto &road : parent_model->node_to_road[this->index]) {
        //using a pointer for new_neighbor because it could be a null pointer (not available with references)
        RouteModel::Node *new_neighbor = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
        if (new_neighbor != nullptr) {
            this->neighbors.push_back(new_neighbor);
        }
    }
}

//To conduct a search, a RoutePlanner class instance will first be initialized starting 
//and ending coordinates provided by the user as float values. However, this presents a problem: 
//these float values might not exactly correspond to any given node on the map.
//FindClosestNode is a method FindClosestNode that accepts two floats and finds the closest node in your model.
RouteModel::Node &RouteModel::FindClosestNode(float x, float y) {
    //creating a temporary node
    Node tempNode;
    tempNode.x = x;
    tempNode.y = y;

    //initilizing the minimum distance found in the search
    //need to start out as max so it can descend
    float min_dist = std::numeric_limits<float>::max();
    int closest_idx;
    float dist;

    //looping over all the roadways
    for (auto &road : Roads()) {
        //check to make sure its not a footway
        if (road.type != Model::Road::Type::Footway) {
            //Loop over each node index in the way that the road belongs to: Ways()[road.way].nodes.
            for (auto way : Ways()[road.way].nodes) {
                dist = tempNode.distance(SNodes()[way]);
                //Update closest_idx and min_dist, if needed
                if (dist < min_dist) {
                    closest_idx = way;
                    min_dist = dist;
                }
            }
        }
    }
    //Return the node from the SNodes() vector using the found index.
    return SNodes()[closest_idx];
}
