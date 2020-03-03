#include "ofApp.h"
#include <queue>
#include "../Tools.h"
#include "../Algorithms.h"
#include <vector>
#include <fstream>
#include <string>
// 
namespace {
	float s_Width = 700;
	//float s_CellSize = 70; // cell height and width
	ofVec2f linePos1(s_MarginLeftX, s_MarginTopY);
	ofVec2f linePosVert(s_MarginLeftX, s_MarginTopY + s_Width);
	ofVec2f linePosHor(s_MarginLeftX + s_Width, s_MarginTopY );

	std::vector<Location> s_circles;
	std::vector<Location> s_Obstacles;
	Location s_Goal;
}
// ====================================
// helpers

std::pair<int, int> getClickGridPos(float x, float y)
{	
	std::pair<int, int> result(4, 4);
	int xi = (x - s_MarginLeftX) / s_CellSize;
	int yi = (y - s_MarginTopY) / s_CellSize;
	std::cout << xi << " " << yi << endl;
	return std::pair<int, int>(xi,yi);
}

// ====================================


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

	std::unordered_set<Location>::iterator it = grid.forests.begin();
	while (it != grid.forests.end())
	{
		s_Obstacles.push_back(*it);
		it++;
	}
	it = grid.mObstacles.begin();
	while (it != grid.mObstacles.end())
	{
		s_Obstacles.push_back(*it);
		it++;
	}
	return grid;
}

// 2 BASIC 
void ExecuteFirstExample()
{
	Graph graf;
	graf.mLinks = {
		 {'S', {'L', 'O'} },
		 {'L', {'E', 'S'} },
		 {'O', {'S', 'C'} },
		 {'E', {'L', 'W', 'P', 'B', 'C'} },
		 {'W', {'E', 'P' } },
		 {'P', {'W', 'E', 'G'} },
		 {'C', {'E','O','U','Z'} },
		 {'X', {'O', 'C', 'Z'} },
		 {'J', {'X', 'R', 'V' } },
		 {'V', {'J', 'F'} },
		 {'Z', {'X', 'C', 'K', 'R'} },
		 {'G', {'P', 'I'} },
		 {'B', {'E','I','U'} },
		 {'I', {'G','B','H'} },
		 {'U', {'B', 'C', 'K', 'H'} },
		 {'K', {'U', 'Z', 'R' , 'H'} },
		 {'H', {'I', 'U', 'K', 'F'} },
		 {'R', {'K', 'Z', 'J', 'F'} },
		 {'F', {'R', 'V', 'H'} },
	};
	graf.isConstMode = true;
	graf.mHeuristic = {
		{ {'A', 'G'}, 20.0f},
		{ {'B', 'G'}, 10.0f},
		{ {'C', 'G'}, 8.0f},
		{ {'G', 'G'}, 0.0f},
	};

	graf.mSinkCost = {
			{ { 'S', 'L'}, 6 }, // s
			{ { 'S', 'O'}, 2 },
			{ { 'O', 'C'}, 5 },
			{ { 'L', 'E'}, 4 },
			{ { 'E', 'W'}, 10},
			{ { 'E', 'P'}, 15},
			{ { 'W', 'P'}, 3 },
			{ { 'P', 'G'}, 11},
			{ { 'E', 'C'}, 7 },
			{ { 'E', 'B'}, 10},
			{ { 'O', 'X'}, 11},
			{ { 'X', 'J'}, 9 },
			{ { 'J', 'V'}, 5 },
			{ { 'X', 'Z'}, 3 },
			{ { 'C', 'Z'}, 8 },
			{ { 'C', 'U'}, 15},
			{ { 'G', 'I'}, 6 },
			{ { 'B', 'I'}, 7 },
			{ { 'I', 'H'}, 7 },
			{ { 'U', 'H'}, 8 },
			{ { 'K', 'H'}, 14},
			{ { 'K', 'R'}, 8 },
			{ { 'K', 'U'}, 8 },
			{ { 'B', 'U'}, 17},
			{ { 'H', 'F'}, 7 },
			{ { 'R', 'F'}, 15},
			{ { 'R', 'Z'}, 12},
			{ { 'R', 'J'}, 3 },
			{ { 'V', 'F'}, 2 },
			{ { 'Z', 'K'}, 5 },
	};

	std::unordered_map<Node, Node, NodeHash, NodeEq> came_fromm;
	std::unordered_map<Node, double, NodeHash, NodeEq> cost_so_farr;

	Dijkstra_Search_1<Graph, Node, NodeHash, NodeEq>
		(graf, 'L', 'F', came_fromm, cost_so_farr);

	//AStar_search_1<Graph, Node, NodeHash, NodeEq>
		//(graf, 'L', 'F', came_fromm, cost_so_farr);

	std::vector<Node> path = reconstruct_path_1('L', 'F', came_fromm);

	int a = 6;
}

void ExecuteGridExample()
{
	GraphWithWeights grid = make_example();
	Location start{ 1, 4 };
	Location goal = s_Goal;
	std::unordered_map<Location, Location> came_from;
	std::unordered_map<Location, double> cost_so_far;
	Dijkstra_Search(grid, start, goal, came_from, cost_so_far);
	draw_grid(grid, 2, nullptr, &came_from);
	std::cout << '\n';
	draw_grid(grid, 3, &cost_so_far, nullptr);
	std::cout << '\n';
	std::vector<Location> path = reconstruct_path(start, goal, came_from);
	draw_grid(grid, 3, nullptr, nullptr, &path);
	s_circles = path;

}

GraphLargeData ofApp::ParseLargeDataSet()
{
	string line;
	string token;
	GraphLargeData gLarge; // fill this values
	// fetchers
	int src = 0, sink = 0, cost = 0;
	vector<int> fetcher;
	std::stringstream stream("");
	ifstream myfile("DataSets/NYC.txt");
	if (myfile.is_open())
	{
		while (getline(myfile, line))
		{
			stream << line;
			while (getline(stream, token, ' '))
			{
				fetcher.push_back(atoi(token.data()));
			}
			// exstract
			src = fetcher[1];
			sink = fetcher[2];
			cost = fetcher[3];
			fetcher.clear();
			stream.clear();
			if (gLarge.mLinks.find(src) != gLarge.mLinks.end())
			{
				// if already exist then enter the sink
				gLarge.mLinks[src].push_back(sink);
			}
			else // if not then enter
			{
				gLarge.mLinks.insert({ src ,{ sink } });
			}

			gLarge.mSinkCost.insert({ {src, sink}, cost });
		}
		myfile.close();
	}

	else cout << "Unable to open file";

	return gLarge;
}

//--------------------------------------------------------------
void ofApp::setup() {
	

	ExecuteFirstExample();
	auto graflarge = ParseLargeDataSet();
	std::unordered_map<int, int, std::hash<int>, intEq> came_fromm;
	std::unordered_map<int, double, std::hash<int>, intEq> cost_so_farr;
	Dijkstra_Search_1<GraphLargeData, int, std::hash<int>, intEq>
		(graflarge, 1900, 219111, came_fromm, cost_so_farr);
	

	//ExecuteGridExample();
	int aaa = 5;
}

//--------------------------------------------------------------
void ofApp::update(){

	// On click follow

	// get click position
	// get screen grid location
	// pass as goal to graph
	// get path and covert to screen space
	// apply seek algo
	

}

//--------------------------------------------------------------
void ofApp::draw(){
	ofSetColor(200, 0, 0);
	DrawGrid();

	for(auto s: s_circles)
		DrawCircleInCell(s.x, s.y);

	ofSetColor(200, 200, 150);
	for (auto s : s_Obstacles)
		DrawCircleInCell(s.x, s.y);

}

// Helpers

// Draw a 10X10 Grid square
void ofApp::DrawGrid()
{
	int offset = 0;
	for (int i = 0; i < 11; i++)
	{
		ofDrawLine(linePos1 + ofVec2f(offset,0), linePosVert + ofVec2f(offset,0));
		ofDrawLine(linePos1 + ofVec2f(0, offset), linePosHor + ofVec2f(0, offset));
		offset += s_CellSize;
	}
}

void ofApp::DrawCircleInCell(int x, int y)
{
	auto pos = getLocalizedOnScreenPosition({ x,y });
	ofVec2f p(pos.first, pos.second);
	ofDrawCircle(p + s_CellSize/2, 30.0f);
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
	auto loc = getQuanizedLocation(x, y);
	s_Goal = loc;
	ExecuteGridExample();
	//s_circles.push_back(loc);
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
