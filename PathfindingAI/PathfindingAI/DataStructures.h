#pragma once
#include <vector>
#include <iostream>
#include <queue>
#include <array>
#include <unordered_set>
#include <unordered_map>

struct Location {
	int x, y;
};

struct Node {
	char id;

	Node() :id('^') {}

	Node(char c) : id(c) {}

	bool operator==(const Node& oth)
	{
		return  id == oth.id;
	}

};

bool operator==(std::pair<Node, Node> a, pair<Node, Node> b)
{
	return a.first == b.first && a.second == b.second;
}

namespace std {
	/* implement hash function to put GridLocation into an unordered_set */
	template <> struct hash<Location> {
		typedef Location argument_type;
		typedef std::size_t result_type;
		std::size_t operator()(const Location& id) const noexcept {
			return std::hash<int>()(id.x ^ (id.y << 4));
		}
	};

	template <> struct hash<Node> {
		typedef Node argument_type;
		typedef std::size_t result_type;
		std::size_t operator()(const Node& id) const noexcept {
			return std::hash<int>()(id.id ^ (id.id << 4));
		}
	};
}



struct NodeHash {
	std::size_t operator()(const Node& id) const noexcept {
		return std::hash<char>()(id.id);
	}
};

struct NodeEq {
	bool operator()(const Node& b, const Node& a) const noexcept {
		return a.id == b.id;
	}
};


struct Graph {
	
	std::unordered_map<Node, std::vector<Node>, NodeHash, NodeEq> mLinks;
	std::vector<pair<Node, Node>> mSinkPair;
	std::vector<double> mCost;

	// get connectors
	std::vector<Node> getNeighbours(const Node &iNode)
	{
		return mLinks[iNode];
	}

	double getCost(const Node &from, const Node &to)
	{
		std::pair<Node, Node> pr(from, to);
		int ct = 0;
		for (int i = 0; i < mSinkPair.size(); i++)
		{
			if (mSinkPair[i] == pr)
				return mCost[i];
		}
		return -1.0f;
	}
};

// Priority Queue
template<typename T, typename priority_t>
struct PriorityQueue {
	typedef std::pair<priority_t, T> PQElement;
	std::priority_queue<PQElement, std::vector<PQElement>,
		std::greater<PQElement>> elements;

	inline bool empty() const {
		return elements.empty();
	}

	inline void put(T item, priority_t priority) {
		elements.emplace(priority, item);
	}

	T get() {
		T best_item = elements.top().second;
		elements.pop();
		return best_item;
	}
};



struct SquareGrid {
	static std::array<Location, 4> mDirections; // four directions
	std::unordered_set<Location> mObstacles; // walls or obstacles
	int mWidth, mHeight; // the size of grid


	SquareGrid(int width_, int height_)
		: mWidth(width_), mHeight(height_) {}

	// check if a point is in range
	bool IsInBounds(Location id) const {
		bool state = 0 <= id.x && id.x < mWidth
			&& 0 <= id.y && id.y < mHeight;

			return state;
	}

	// whether a valid node or obstacle/wall
	bool IsValid(Location id) const {
		return mObstacles.find(id) == mObstacles.end();
	}

	// return all neighbours
	std::vector<Location> getNeighbours(Location id) const {
		std::vector<Location> results;

		for (Location dir : mDirections) {
			Location next{ id.x + dir.x, id.y + dir.y };
			if (IsInBounds(next) && IsValid(next)) {
				results.push_back(next);
			}
		}

		if ((id.x + id.y) % 2 == 0) {
			// aesthetic improvement on square grids
			std::reverse(results.begin(), results.end());
		}

		return results;
	}
};

// The four direction for neighbour
std::array<Location, 4> SquareGrid::mDirections =
{ Location{1, 0}, Location{0, -1}, Location{-1, 0}, Location{0, 1} };

// ------------------- Helpers ----------------

bool operator == (Location a, Location b) {
	return a.x == b.x && a.y == b.y;
}

bool operator != (Location a, Location b) {
	return !(a == b);
}

bool operator < (Location a, Location b) {
	return std::tie(a.x, a.y) < std::tie(b.x, b.y);
}

bool operator < (Node a, Node b) {
	return (int)a.id < (int)b.id;
}

std::basic_iostream<char>::basic_ostream& operator<<(std::basic_iostream<char>::basic_ostream& out, const Location& loc) {
	out << '(' << loc.x << ',' << loc.y << ')';
	return out;
}
// ----------------------------------------------

struct GraphWithWeights : SquareGrid {
	// forest are dense areas with very high cost/weights
	std::unordered_set<Location> forests;
	GraphWithWeights(int w, int h) : SquareGrid(w, h) {}
	
	double getCost(Location from_node, Location to_node) const
	{
		return forests.find(to_node) != forests.end() ? 5 : 1;
	}
};

// add walls
void add_rect(SquareGrid& grid, int x1, int y1, int x2, int y2) {
	for (int x = x1; x < x2; ++x) {
		for (int y = y1; y < y2; ++y) {
			grid.mObstacles.insert(Location{ x, y });
		}
	}
}

void add_Obstacle(SquareGrid& grid, Location iLoc)
{
	grid.mObstacles.insert(iLoc);
}


// This outputs a grid. Pass in a distances map if you want to print
// the distances, or pass in a point_to map if you want to print
// arrows that point to the parent location, or pass in a path vector
// if you want to draw the path.
template<class AGraph>
void draw_grid(const AGraph& graph, int field_width,
	std::unordered_map<Location, double>* distances = nullptr,
	std::unordered_map<Location, Location>* point_to = nullptr,
	std::vector<Location>* path = nullptr) {
	for (int y = 0; y != graph.mHeight; ++y) {
		for (int x = 0; x != graph.mWidth; ++x) {
			Location id{ x, y };
			std::cout << std::left << std::setw(field_width);
			if (graph.mObstacles.find(id) != graph.mObstacles.end()) {
				std::cout << std::string(field_width, '#');
			}
			else if (point_to != nullptr && point_to->count(id)) {
				Location next = (*point_to)[id];
				if (next.x == x + 1) { std::cout << "> "; }
				else if (next.x == x - 1) { std::cout << "< "; }
				else if (next.y == y + 1) { std::cout << "v "; }
				else if (next.y == y - 1) { std::cout << "^ "; }
				else { std::cout << "* "; }
			}
			else if (distances != nullptr && distances->count(id)) {
				std::cout << (*distances)[id];
			}
			else if (path != nullptr && find(path->begin(), path->end(), id) != path->end()) {
				std::cout << '@';
			}
			else {
				std::cout << '.';
			}
		}
		std::cout << '\n';
	}
}



