#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes
    // to the starting and ending coordinates. Store the nodes you find in the
    // RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
    }

// TODO 3: Implement the CalculateHValue method.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*end_node);
  }

// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
  for (RouteModel::Node *neighbor : current_node->neighbors) {
    neighbor->parent = current_node;
    neighbor->h_value = CalculateHValue(neighbor);
    neighbor->g_value =
        current_node->g_value + current_node->distance(*neighbor);
    open_list.push_back(neighbor);
    neighbor->visited = true;
  };
}

// TODO 5: Complete the NextNode method to sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
  float cost = open_list[0]->h_value + open_list[0]->g_value;
  float lowestcost = cost;
  int nodetochoose = 0;
  for (int i = 1; i <= open_list.size(); i++) {
    cost = open_list[i]->h_value + open_list[i]->g_value;
    if (cost < lowestcost) {
      lowestcost = cost;
      nodetochoose = i;
    }
  }
  open_list.erase(open_list.begin()+nodetochoose);
  return open_list[nodetochoose];
}

// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    RouteModel::Node* tempnode = current_node;
    while (tempnode != start_node) {
      distance = distance + tempnode->distance(*tempnode->parent);
      path_found.insert(path_found.begin(), *tempnode);
      tempnode = tempnode->parent;
    }
    path_found.insert(path_found.begin(), *tempnode);
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.

}