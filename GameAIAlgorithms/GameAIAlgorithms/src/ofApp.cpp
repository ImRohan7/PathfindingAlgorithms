#include "ofApp.h"
#include <queue>
#include "../Tools.h"
#include "../Algorithms.h"
#include <vector>
#include <fstream>
#include <string>
#include "../KinemSeek.h"
#include "../AISystem.h"
#include "../Decision Making/DTree.h"
#include "../Decision Making/CustomDecisions.h"

namespace {
	// select Decion Making algo type
	DecisionAlgoType s_AlgoType = DecisionAlgoType::DecisionTree;
	
	// for grid
	GraphWithWeights s_Grid(1,1);
	
	ofVec2f linePos1(s_MarginLeftX, s_MarginTopY);
	ofVec2f linePosVert(s_MarginLeftX, s_MarginTopY + s_Width);
	ofVec2f linePosHor(s_MarginLeftX + s_Width, s_MarginTopY );

	std::vector<Location> s_Obstacles;
	Location s_GoalA, s_GoalB;
	Location s_Start({ 1,4 });

	// Kinematic 
	//physics::Kinematic lead; // leader
	//physics::Kinematic targt;
	AI::KinemSeek _player;
	AI::KinemSeek _mosnsterA;
	physics::SteeringOutput steer;
	int s_curTargetPlayer = 0; // target to follow
	int s_curTargetMonster = 0; // target to follow
	std::vector<Location> s_PathcirclesPlayer; // path
	std::vector<Location> s_PathcirclesMonster; // path

	// Decision Making variables
	float s_MaxVel = 2; // 3
	float s_MaxAcceleration = 6; // 12
	int s_ClickCounter = 0;
}

// ====================================


//----------
// Example setups
// 1
GraphWithWeights createGridGraph() {
	GraphWithWeights grid(20, 20);
	//add_rect(grid, 1, 7, 4, 9); // walls
	typedef Location L;
	// dense areas almost as obstacles
	grid.forests = std::unordered_set<Location>{
	  L{3, 4}, L{3, 5}, L{4, 1}, L{4, 2},
	  L{4, 3},
	 	  L{5, 3}, L{5, 5}, L{5, 6},
	  L{5, 7}, L{6, 12}, L{6, 3},
	  L{16, 6}, L{6, 7},
	  L{7, 3}, L{7, 5},
	   L{14, 3},
		  L{5, 3}, L{15, 15}, L{15, 16},
	  L{15, 7}, L{16, 12}, L{16, 13},
	  L{16, 1}, L{16, 17},
	  L{7, 19}, L{17, 15}
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

// 3 Grid
void ofApp::ExecuteGridExample()
{
	Location start = getQuanizedLocation(
		_player.mCharacter.mPosition.x, _player.mCharacter.mPosition.y);
	Location goal = s_GoalA;
	std::unordered_map<Location, Location> came_from;
	std::unordered_map<Location, double> cost_so_far;
	if(s_AlgoType == AlgoType::DjKstra)
		Dijkstra_Search(s_Grid, start, goal, came_from, cost_so_far);
	else
		a_star_search(s_Grid, start, goal, came_from, cost_so_far);
		
	//draw_grid(grid, 2, nullptr, &came_from);
	//std::cout << '\n';
	//draw_grid(grid, 3, &cost_so_far, nullptr);
	//std::cout << '\n';
	std::vector<Location> path = reconstruct_path(start, goal, came_from);
	//draw_grid(grid, 3, nullptr, nullptr, &path);
	s_PathcirclesPlayer = path;

}

// decision making
void ofApp::RunDecisionTree()
{
	// running two algos
	Location start = getQuanizedLocation(
		_player.mCharacter.mPosition.x, _player.mCharacter.mPosition.y);
	Location goalA = s_GoalA;
	Location goalB = s_GoalB;
	std::unordered_map<Location, Location> came_fromA;
	std::unordered_map<Location, Location> came_fromB;
	std::unordered_map<Location, double> cost_so_farA;
	std::unordered_map<Location, double> cost_so_farB;
	
	a_star_search(s_Grid, start, goalA, came_fromA, cost_so_farA);
	a_star_search(s_Grid, start, goalB, came_fromB, cost_so_farB);
	
	std::vector<Location> pathA = reconstruct_path(start, goalA, came_fromA);
	std::vector<Location> pathB = reconstruct_path(start, goalB, came_fromB);

	// ==========================
	// variables
	int sizeA = pathA.size();
	int sizeB = pathB.size();

	float DistA = heuristic_2(start, goalA);
	float DistB = heuristic_2(start, goalB);
	
	// Decision Trees
	ChooseAlgo* A1 = new ChooseAlgo(sizeA,sizeB);
	ChooseSpeed* BT = new ChooseSpeed(DistA, 10); // true node
	ChooseSpeed* BF = new ChooseSpeed(DistB, 10); // false node
	Decision* DT = new Decision();
	Decision* DF = new Decision();
	DT->m_HasAction = true;
	DT->mVel = 2;
	DT->mAccel = 6;
	
	DF->m_HasAction = true;
	DF->mVel = 4l;
	DF->mAccel = 15;

	A1->m_BranchTrue = BT;
	A1->m_BranchFalse = BF;
	BT->m_BranchTrue = DT;
	BT->m_BranchFalse = DF;
	BF->m_BranchTrue = DT;
	BF->m_BranchFalse = DF;

	Decision* D = A1->makeADecision();

	s_MaxVel = D->mVel;
	s_MaxAcceleration = D->mAccel;
	_player.mCharacter.mMaxVel = s_MaxVel;
	_player.mMaxAccel = s_MaxAcceleration;

	s_PathcirclesPlayer = (sizeA < sizeB) ? pathA : pathB;
}


// Helpers ==============
void ResetCharacterForFollow()
{
	s_curTargetPlayer = 0;
	
	_player.mCharacter.mVelocity = ofVec2f(0,0);
//	seek.mCharacter.mPosition = getAbsoluteObjectPosition(s_Start);
	_player.mTarget.mPosition = _player.mCharacter.mPosition;
}

// add forest
void ofApp::addForest(Location l)
{
	// check if already there
	if (s_Grid.forests.find(l) != s_Grid.forests.end())
	{
		s_Grid.forests.erase(s_Grid.forests.find(l));
	}
	else
	{
		s_Grid.forests.insert(l);
	}
	
}

// choose target and speed
void ofApp::MakeDecision_ChooseTarget()
{
	// cre
}

//--------------------------------------------------------------
void ofApp::setup() {

	switch (s_AlgoType)
	{
	
	case DecisionAlgoType::DecisionTree:
		// follow setup
		s_Grid = createGridGraph();
		ExecuteGridExample();
		_player = AI::KinemSeek(physics::Kinematic(), physics::Kinematic(), 8.0f);
		_player.mCharacter.mMaxVel = s_MaxVel;
		_player.mMaxAccel = s_MaxAcceleration;
		_player.mMaxSpeed = 6;
		_player.mSlowRadArrive = 25;
		_player.mTargetRadArrive = 10;
		_player.mTimeTotargetArrive = 0.4f;

		ResetCharacterForFollow();
		break;

	default:
		break;
	}

}

//--------------------------------------------------------------
void ofApp::update(){

	// update target
	if (s_AlgoType == DecisionAlgoType::DecisionTree)
	{
		if (_player.mSlowRadReached)
		{
			_player.mSlowRadReached = false;
			// change target position
			s_curTargetPlayer++;
			if (s_curTargetPlayer >= s_PathcirclesPlayer.size())
				s_curTargetPlayer--;
			auto newTarget = getAbsoluteObjectPosition(s_PathcirclesPlayer[s_curTargetPlayer]);
			_player.mTarget.mPosition = newTarget;
		}

		//  get steering and update 
		steer = _player.getSteeringForArrival();
		float tOr = atan2(_player.mCharacter.mVelocity.y, _player.mCharacter.mVelocity.x);
		steer.mAngular = AISystem::getSteeringFor_Align(
			tOr, _player.mCharacter.mOrientation,
			1.5, 0.3f, 2.5).mAngular;
		_player.mCharacter.update(steer, ofGetLastFrameTime()); // update 
	}

}

//--------------------------------------------------------------
void ofApp::draw(){
	// Grid Stuff
	if (s_AlgoType == DecisionAlgoType::DecisionTree)
	{
		ofSetColor(200, 0, 0);
		DrawGrid();

		ofSetColor(83, 228, 250); // Yellow circles path
		for (auto s : s_PathcirclesPlayer)
			DrawCircleInCell(s.x, s.y);
		
		ofSetColor(10, 223, 60); // Green circles First
		DrawCircleInCell(s_PathcirclesPlayer[0].x, s_PathcirclesPlayer[0].y);

		auto size = s_PathcirclesPlayer.size();
		ofSetColor(0, 0, 250); // Blue Goals
		DrawCircleInCell(s_GoalA.x, s_GoalA.y );
		DrawCircleInCell(s_GoalB.x, s_GoalB.y);

		ofSetColor(240, 0, 0);
		for (auto s : s_Grid.forests)
			DrawCircleInCell(s.x, s.y);


		// follow stuff
		ofSetColor(250, 0, 150);
		ofNoFill();
		//ofDrawCircle(seek.mTarget.mPosition, seek.mSlowRadArrive); // slow rad
		//ofDrawCircle(seek.mTarget.mPosition, seek.mTargetRadArrive); // target rad
		ofFill();
		//drawBoid(seek.mTarget.mPosition, seek.mTarget.mOrientation);
		ofDrawLine(_player.mCharacter.mPosition, _player.mCharacter.mPosition + 10 * steer.mLinear); // acceleration line

		drawBoid(_player.mCharacter.mPosition, _player.mCharacter.mOrientation,
			ofColor(250, 0, 150));
	}
}

// Helpers
// rotat and draw BOid with triangle
void ofApp::drawBoid(ofVec2f &pos, float &ori, ofColor &clr)
{
	// translate
	ofTranslate(pos);
	// rotate
	ofRotateZRad(ori); // rotate
	// draw
	ofSetColor(clr);
	ofDrawCircle(ofVec2f(0, 0), 10);
	ofSetColor(150, 200, 0);
	ofDrawTriangle(ofVec2f(5, -10), ofVec2f(5, 10), ofVec2f(15, 0));
	// reverse rotate
	ofRotateZRad(-ori); // rotate
	// translate back
	ofTranslate(-pos);

}

// Draw a 10X10 Grid square
void ofApp::DrawGrid()
{
	int offset = 0;
	for (int i = 0; i < s_boxPerLine+1; i++)
	{
		ofDrawLine(linePos1 + ofVec2f(offset,0), linePosVert + ofVec2f(offset,0));
		ofDrawLine(linePos1 + ofVec2f(0, offset), linePosHor + ofVec2f(0, offset));
		offset += s_CellSize;
	}
}

void ofApp::DrawCircleInCell(int x, int y)
{
	auto pos = getLocalizedOnScreenPosition({ x,y });
	ofVec2f po(pos.first, pos.second);
	ofDrawCircle(po + s_CellSize/2, 15.0f);
}

//--------------------------------------------------------
// MOUSE PRESSED
//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{
	auto loc = getQuanizedLocation(x, y);
	
	if (button == 2) // right click add forest
	{
		addForest(loc);
	}
	else
	{
		s_ClickCounter++;
		if (s_ClickCounter == 1)
			s_GoalA = loc;
		if (s_ClickCounter == 2)
		{
			s_GoalB = loc;
			s_ClickCounter = 0;

			RunDecisionTree();
			ResetCharacterForFollow(); // reset
			//s_circles.push_back(loc);
		}
	}
}

// 1 BASIC 
void ofApp::ExecuteCampusMap()
{
	Node start = 'L', goal = 'H';
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
	graf.mRandomRange = 20;
	graf.mHeuristic = {
		// it is random in range
	};

	graf.mSinkCost = {
			{ { 'S', 'L'}, 6 }, // s
			{ { 'S', 'O'}, 2 },
			{ { 'O', 'C'}, 5 },
			{ { 'L', 'E'}, 4 },
			{ { 'E', 'W'}, 10},
			{ { 'E', 'P'}, 11},
			{ { 'W', 'P'}, 3 },
			{ { 'P', 'G'}, 11},
			{ { 'E', 'C'}, 7 },
			{ { 'E', 'B'}, 8},
			{ { 'O', 'X'}, 5},
			{ { 'X', 'J'}, 9 },
			{ { 'J', 'V'}, 5 },
			{ { 'X', 'Z'}, 3 },
			{ { 'C', 'Z'}, 8 },
			{ { 'C', 'U'}, 10},
			{ { 'G', 'I'}, 6 },
			{ { 'B', 'I'}, 7 },
			{ { 'I', 'H'}, 7 },
			{ { 'U', 'H'}, 8 },
			{ { 'K', 'H'}, 4 },
			{ { 'K', 'R'}, 6 },
			{ { 'K', 'U'}, 8 },
			{ { 'B', 'U'}, 3 },
			{ { 'H', 'F'}, 7 },
			{ { 'R', 'F'}, 5 },
			{ { 'R', 'Z'}, 9 },
			{ { 'R', 'J'}, 3 },
			{ { 'V', 'F'}, 2 },
			{ { 'Z', 'K'}, 5 },
	};

	std::unordered_map<Node, Node, NodeHash, NodeEq> came_fromm;
	std::unordered_map<Node, double, NodeHash, NodeEq> cost_so_farr;

	if (s_AlgoType == AlgoType::DjKstra)
	{
		Dijkstra_Search_1<Graph, Node, NodeHash, NodeEq>
			(graf, start, goal, came_fromm, cost_so_farr);
	}
	else
	{
		AStar_search_1<Graph, Node, NodeHash, NodeEq>
			(graf, start, goal, came_fromm, cost_so_farr);
	}
	// construct and print path
	std::vector<Node> path = reconstruct_path_1(start, goal, came_fromm);
	cout << "Path:\n";
	for (auto p : path)
	{
		std::cout << p.id << " -> ";
	}
}

// 2 Large
GraphLargeData ofApp::ParseLargeDataSet()
{
	string line;
	string token;
	GraphLargeData gLarge; // fill this values
	// fetchers
	int src = 0, sink = 0, cost = 0;
	vector<int> fetcher;
	std::stringstream stream("");
	ifstream myfile("DataSets/rome.txt");
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

void ofApp::ExecuteLargeDataSets()
{
	auto graflarge = ParseLargeDataSet();

	std::unordered_map<int, int, std::hash<int>, intEq> came_fromm;
	std::unordered_map<int, double, std::hash<int>, intEq> cost_so_farr;

	if (s_AlgoType == AlgoType::DjKstra)
	{
		Dijkstra_Search_1<GraphLargeData, int, std::hash<int>, intEq>
			(graflarge, 1900, 219111, came_fromm, cost_so_farr);
	}
	else {
		AStar_search_1<GraphLargeData, int, std::hash<int>, intEq>
			(graflarge, 1900, 219111, came_fromm, cost_so_farr);
	}
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
