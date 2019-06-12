#include "route_model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
	for (int counter=0; counter < (this->Nodes()).size(); ++counter){
        m_Nodes.push_back(Node(counter, this, (this->Nodes())[counter]));
    }
    CreateNodeToRoadHashmap();
}

void RouteModel::CreateNodeToRoadHashmap() {
	for(const Model::Road &road : Roads()) {
    	if (road.type != Model::Road::Type::Footway){
        	for(int node_idx: Ways()[road.way].nodes) {
            	if (node_to_road.find(node_idx)==node_to_road.end()) {
                  	node_to_road[node_idx] = std::vector<const Model::Road*> {};
                }
                node_to_road[node_idx].push_back(&road);
            }
        }
    }
}

RouteModel::Node* RouteModel::Node::FindNeighbor(std::vector<int> node_indices){
	Node *closest_node = nullptr;
    Node node;
    for (int node_index: node_indices) {
    	node = parent_model->SNodes()[node_index];
        if (!node.visited && this->distance(node) != 0) {
        	if (closest_node == nullptr || (this->distance(*closest_node) > this->distance(node))){
            	closest_node = & parent_model->SNodes()[node_index];
            } 
        }
    }
    return closest_node;
}

void RouteModel::Node::FindNeighbors() {
	for (auto &road: parent_model->node_to_road[this->index]) {
    	RouteModel::Node* found = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
        if (found) {
        	this->neighbors.emplace_back(found);
        }
    }
}

RouteModel::Node& RouteModel::FindClosestNode(float x, float y) {
	Node input;
  	input.x = x;
    input.y = y;
    float min_dist = std::numeric_limits<float>::max();
    int closest_idx;
    for (auto &road: Roads()) {
    	if (road.type != Model::Road::Type::Footway) {
        	for (int idx: Ways()[road.way].nodes) {
            	if (min_dist > input.distance(SNodes()[idx])) {
                	min_dist = input.distance(SNodes()[idx]);
                    closest_idx = idx;
                }
            }
        }
    }
    return SNodes()[closest_idx];
}