#include "ofApp.h"
#include <queue>
#include "../Algorithms.h"

//----------
// Example
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

//--------------------------------------------------------------
void ofApp::setup() {

	/*GraphWithWeights grid = make_example();
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
	draw_grid(grid, 3, nullptr, nullptr, &path);*/

	//auto a = heuristic_1(GridLocation({ 1,1 }), GridLocation({ 6,2 }));
	//auto b =heuristic_2(GridLocation({ 1,1 }), GridLocation({ 6,2 }));
	
	Node a = Node('A');
	Node b = Node('B');
	Node c = Node('C');
	Node g = Node('G');

	Graph graf;
	graf.mLinks = { {
		 {a, {'B', 'C'}},
		 {b, {'A', 'C', 'G'}},
		 {c, {'A', 'B', 'G'}},
		 {g, {'C', 'B'}},
	} };

	graf.mSinkPair = { 
		{ 'A', 'B'},
		{ 'A', 'C'},
		{ 'B', 'A'},
		{ 'B', 'C'},
		{ 'B', 'G'},
		{ 'C', 'A'},
		{ 'C', 'B'},
		{ 'C', 'G'},
		{ 'G', 'C'},
		{ 'G', 'B'},
	};

	graf.mCost = {
		20, 40,
		20, 10, 40,
		40, 10, 5,
		5, 40
	};

	auto nei = graf.getNeighbours('C');
	//std::unordered_map<Node, Node> came_fromm;
	//std::unordered_map<Node, double> cost_so_farr;
//	std::unordered_map<Location, double> cost_so_far;

	//AAStar_search(graf, 'A', 'G', came_fromm, cost_so_farr);
	//AAAStar_search( 'A', 'G');
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
