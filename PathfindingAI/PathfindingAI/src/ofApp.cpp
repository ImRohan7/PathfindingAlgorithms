#include "ofApp.h"
#include <queue>
#include "../Algorithms.h"

//----------
// Example setups
// 1
GraphWithWeights make_example() {
	GraphWithWeights grid(10, 10);
	add_rect(grid, 1, 7, 4, 9); // walls
	typedef Location L;
	// dense areas almost as obstacles
	grid.forests = std::unordered_set<Location>{
	  L{3, 4}, L{3, 5}, L{4, 1}, L{4, 2},
	  L{4, 3}, L{4, 4}, L{4, 5}, L{4, 6},
	  L{4, 7}, L{4, 8}, L{5, 1}, L{5, 2},
	  L{5, 3}, L{5, 4}, L{5, 5}, L{5, 6},
	  L{5, 7}, L{5, 8}, L{6, 2}, L{6, 3},
	  L{6, 4}, L{6, 5}, L{6, 6}, L{6, 7},
	  L{7, 3}, L{7, 4}, L{7, 5}
	};
	return grid;
}

// 2
void ExecuteBasicExample()
{
	Graph graf;
	graf.mLinks = {
		 {'A', {'B', 'C'}},
		 {'B', {'A', 'C', 'G'}},
		 {'C', {'A', 'B', 'G'}},
		 {'D', {'C', 'B'}},
	};

	graf.mHeuristic = {
		{ {'A', 'G'}, 20.0f},
		{ {'B', 'G'}, 10.0f},
		{ {'C', 'G'}, 8.0f},
		{ {'G', 'G'}, 0.0f},

	};

	graf.mSinkCost = {
		{ { 'A', 'B'}, 20 },
		{ { 'A', 'C'}, 40 },
		{ { 'B', 'A'}, 20 },
		{ { 'B', 'C'}, 10 },
		{ { 'B', 'G'}, 40 },
		{ { 'C', 'A'}, 40 },
		{ { 'C', 'B'}, 10 },
		{ { 'C', 'G'}, 5 },
		{ { 'G', 'C'}, 5 },
		{ { 'G', 'B'}, 40 },
	};

	std::unordered_map<Node, Node, NodeHash, NodeEq> came_fromm;
	std::unordered_map<Node, double, NodeHash, NodeEq> cost_so_farr;

	AStar_search_1(graf, 'A', 'G', came_fromm, cost_so_farr);
	//	Dijkstra_Search_1(graf, 'A', 'G', came_fromm, cost_so_farr);

}

void ExecuteGridExample()
{
	GraphWithWeights grid = make_example();
	Location start{ 1, 4 };
	Location goal{ 8, 5 };
	std::unordered_map<Location, Location> came_from;
	std::unordered_map<Location, double> cost_so_far;
	Dijkstra_Search(grid, start, goal, came_from, cost_so_far);
	draw_grid(grid, 2, nullptr, &came_from);
	std::cout << '\n';
	draw_grid(grid, 3, &cost_so_far, nullptr);
	std::cout << '\n';
	std::vector<Location> path = reconstruct_path(start, goal, came_from);
	draw_grid(grid, 3, nullptr, nullptr, &path);

}

//--------------------------------------------------------------
void ofApp::setup() {

	
	int aaa = 5;
}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw(){

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
