#include "route_planner.h"
#include <algorithm> // sort and reverse
#include <iostream> // for debug
using std::sort;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    this->start_node = &m_Model.FindClosestNode( start_x, start_y );
    this->end_node = &m_Model.FindClosestNode( end_x, end_y );

}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // method - so this object as a paramter is implicit
    // this, node --> float # this object (RoutePlanner has the end node (goal) as an attribute )
    return node->distance( *this->end_node );
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for ( auto node : current_node->neighbors ) {
        node->parent = current_node;
        node->h_value = this->CalculateHValue( node );
        node->g_value = current_node->g_value + node->distance( *current_node );    // could also use call on current_node?
        node->visited = true;   // but, we never make use of this. Does the compiler report that?
        this->open_list.push_back( node );
    }
}

bool Compare_nodes( RouteModel::Node *node1, RouteModel::Node *node2 ) {
    // *node, *node --> bool  // if g+h of node1 > that of node2
    return node1->g_value + node1->h_value > node2->g_value + node2->h_value;
}

// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    // this --> node*
    RouteModel::Node *best;
    sort(this->open_list.begin(), this->open_list.end(), Compare_nodes ); // was getting lvalue required as decrement operand 
                                                                    // and decrement of read only location with this, removed, add..mystery..
    best = this->open_list.back();
    this->open_list.pop_back();    // this node has the lowest cost, and we're proceeding with it, so remove it
    return best;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    // std::cout << "In ConstructFinalPath\n";
    // std::cout << "Retrace, starting with " << current_node << "\n";
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    int i = 0;

    // TODO: Implement your solution here.
    RouteModel::Node *parent = current_node->parent;
    path_found.push_back( *current_node );          // mistake made : current_node instead of *current_node
    while( parent && i++ < 33100 ){
        distance += current_node->distance( *parent );  // mistake made : parent instead of *parent
        path_found.push_back( *parent );          // mistake made : current_node instead of *current_node
        // std::cout  << parent << "\n";
        // std::cout << current_node << "\n";

        current_node = parent;
        parent = current_node->parent;


    }
    // std::cout << i << "\n";
    std::reverse( path_found.begin(), path_found.end() );

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
    // the start and end are created in the RoutePlanner instance by the constructor 
    RouteModel::Node *current_node = start_node;
    int i = 0;
    // std::cout << "starting.. and end_node is " << end_node << "\n";
    // std::cout << "parent of start is " << start_node->parent << "\n";
    // TODO: Implement your solution here.
    while( current_node != end_node ){  // how do you know this will work?
        i++;
        if( i > -1 ) { // 33090 - to get the last 9 of 33099
            // std::cout << current_node << "\n";
        }
        AddNeighbors( current_node );
        current_node = NextNode();      // how are you referring to the current object? Just doesn't feel right..
    }
    std::cout << current_node << "   ... and done\n";
    std::cout << "Parent of end " << current_node->parent << "\n";
    std::cout << "hops : " << i << "\n";
    m_Model.path = ConstructFinalPath( current_node );
}