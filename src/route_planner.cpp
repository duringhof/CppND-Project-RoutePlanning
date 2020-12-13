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
    if (neighbor->visited == false) {
      neighbor->parent = current_node;
      neighbor->h_value = CalculateHValue(neighbor);
      neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
      open_list.push_back(neighbor);
      neighbor->visited = true;
    }
  }
}

// TODO 5: Complete the NextNode method to sort the open list and return the
// next node.
bool Compare(RouteModel::Node* a, RouteModel::Node* b) {
    float f1 = a->g_value + a->h_value;
    float f2 = b->g_value + b->h_value;
    return f1 > f2;
}

RouteModel::Node *RoutePlanner::NextNode() {
  sort(open_list.begin(), open_list.end(), Compare);
  return open_list.back();
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
void RoutePlanner::AStarSearch() { 
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    open_list.push_back(start_node);
    start_node->visited = true;

    while (open_list.size() > 0) {
      current_node = NextNode();
      open_list.pop_back();

      if (current_node->x == end_node->x &&
          current_node->y == end_node->y) {
        m_Model.path = ConstructFinalPath(current_node);
        break;
      }

      AddNeighbors(current_node);
    }
}