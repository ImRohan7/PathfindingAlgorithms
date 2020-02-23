#pragma once
#include "DataStructures.h"

//========= Hueristic methods =================

// 1 : Manhattan Distance
double heuristic_1(Location a, Location b) {
	// calc diff of x and y
	double x = std::abs(a.x - b.x);
	double y = std::abs(a.y - b.y);
	
	return x + y;
}

// 2 : Straight Line
double heuristic_2(Location a, Location b)
{
	// we use pythagoras theorem z^2 = x^2 + y^2;
	double x = std::abs(a.x - b.x);
	double y = std::abs(a.y - b.y);
	x *= x;
	y *= y;
	double z = sqrt(x + y);
	return z;
}

// ================================================
// =======	ALGORITHMS

// Dijkstra
void Dijkstra_Search
(GraphWithWeights graph,
	Location start,
	Location goal,
	std::unordered_map<Location, Location>& came_from,
	std::unordered_map<Location, double>& cost_so_far)
{
	PriorityQueue<Location, double> container; // open list
	container.put(start, 0);

	// add start node into open list
	came_from[start] = start; 
	cost_so_far[start] = 0; // close list

	while (!container.empty()) {
		Location current = container.get(); // get a new node and explore neightbours

		if (current == goal) {
			break;	// if we reached the goal already
		}

		for (Location next : graph.neighbors(current)) {
			// cal cost
			double new_cost = cost_so_far[current] + graph.cost(current, next);
			if (cost_so_far.find(next) == cost_so_far.end()
				|| new_cost < cost_so_far[next]) {
				// if it is not inside the closed list 
				// OR the new cost is lower(if its already inside)
				cost_so_far[next] = new_cost; // update / insert node with cost
				came_from[next] = current;	// update path .. how we going to the next node
				container.put(next, new_cost); // add the neighbour into open list to explore further
			}
		}
	}
}

// A star
void AStar_search
(GraphWithWeights graph,
	Location start,
	Location goal,
	std::unordered_map<Location, Location>& came_from,
	std::unordered_map<Location, double>& cost_so_far)
{
	PriorityQueue<Location, double> container;
	container.put(start, 0);

	came_from[start] = start;
	cost_so_far[start] = 0;

	while (!container.empty()) {
		Location current = container.get();

		if (current == goal) {
			break;
		}

		for (Location next : graph.neighbors(current)) {
			double new_cost = cost_so_far[current] + graph.cost(current, next);
			if (cost_so_far.find(next) == cost_so_far.end()
				|| new_cost < cost_so_far[next]) {
				cost_so_far[next] = new_cost;
				double priority = new_cost + heuristic_1(next, goal);
				container.put(next, priority);
				came_from[next] = current;
			}
		}
	}
}

// make graph from paths
template<typename Location>
std::vector<Location> reconstruct_path(
	Location start, Location goal,
	std::unordered_map<Location, Location> came_from
) {
	std::vector<Location> path;
	Location current = goal;
	while (current != start) {
		path.push_back(current);
		current = came_from[current];
	}
	path.push_back(start); // optional
	std::reverse(path.begin(), path.end());
	return path;
}

