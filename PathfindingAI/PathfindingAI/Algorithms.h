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

template <class TGraph, class Point, class Hash, class Eq>
void AStar_search_1
(TGraph graph,
	const Point& start,
	const Point& goal,
	std::unordered_map<Point, Point, Hash, Eq>& came_from,
	std::unordered_map<Point, double, Hash, Eq>& cost_so_far)
{
	PriorityQueue<Point, double> container;
	container.put(start, 0);

	while (!container.empty()) {
		Point current = container.get();

		if (current == goal) {
			break;
		}

		for (Point next : graph.getNeighbours(current)) {
			double new_cost = cost_so_far[current] + graph.getCost(current, next);
			if (cost_so_far.find(next) == cost_so_far.end()
				|| new_cost < cost_so_far[next]) {
				cost_so_far[next] = new_cost;
				double priority = new_cost + graph.getHueristic(next, goal);
				container.put(next, priority);
				came_from[next] = current;
			}
		}
	}
}

// DijKstra
template <class TGraph, class Point, class Hash, class Eq>
void Dijkstra_Search_1
(TGraph graph,
	const Point& start,
	const Point& goal,
	std::unordered_map<Point, Point, Hash, Eq>& came_from,
	std::unordered_map<Point, double, Hash, Eq>& cost_so_far)
{
	PriorityQueue<Point, double> container; // open list
	container.put(start, 0);

	// add start node into open list
	came_from[start] = start;
	cost_so_far[start] = 0; // close list

	while (!container.empty()) {
		Point current = container.get(); // get a new node and explore neightbours

		if (current == goal) {
			break;	// if we reached the goal already
		}

		for (Point next : graph.getNeighbours(current)) {
			// calculate cost
			double new_cost = cost_so_far[current] + graph.getCost(current, next);
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

std::vector<Node> reconstruct_path_1(
	Node start, Node goal,
	std::unordered_map<Node, Node, NodeHash, NodeEq> came_from) 
{
	std::vector<Node> path;
	Node current = goal;
	while (current.id != start.id) {
		path.push_back(current);
		current = came_from[current];
	}
	path.push_back(start); // optional
	std::reverse(path.begin(), path.end());
	return path;
}

// Dijkstra template for any graph
template<class AGraph, class ANode>
void Dijkstra_Search
(AGraph graph,
	ANode start,
	ANode goal,
	std::unordered_map<ANode, ANode>& came_from,
	std::unordered_map<ANode, double>& cost_so_far)
{
	PriorityQueue<ANode, double> container; // open list
	container.put(start, 0);

	// add start node into open list
	came_from[start] = start;
	cost_so_far[start] = 0; // close list

	while (!container.empty()) {
		ANode current = container.get(); // get a new node and explore neightbours

		if (current == goal) {
			break;	// if we reached the goal already
		}

		for (ANode next : graph.getNeighbours(current)) {
			// calculate cost
			double new_cost = cost_so_far[current] + graph.getCost(current, next);
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