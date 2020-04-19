#include "ofApp.h"
#include <queue>
#include "../Tools.h"
#include "../Algorithms.h"
#include <vector>
#include <fstream>
#include <string>
#include "../AISystem.h"
#include "../Decision Making/DTree.h"
#include "../Decision Making/CustomDecisions.h"
#include "../Decision Making/BehaviorTrees.h"
#include "../PathFollower.h"


// helper fun
bool Is_CloseToPlayer(float dist);
std::vector<Location> getPathFortarget(Location start, Location goal);
std::vector<Location> getRandomPathFortarget(Location start);
float getDistancebetweenPlayandMonster();
void resetBehavTree();

namespace {
	// Kinematic 
	Follower _player;
	Follower _monster;
	vector<Location> _Traps;
	bool _IsTrapped = false;
	//// **************************************************
	//// functions

	//  Monster Roam while keeping a safe distance
	class RoamAway : public Task {
	public:
		bool RunTask() override
		{
			if (Is_CloseToPlayer(600))
			{
				return false;
			}
			else {
				if (_monster.m_IsTargetReached)
				{
					_monster.Reset(); // reset
					int x = rand() % 20; // generate random target
					int y = rand() % 20;
					Location goal(x, y);
					Location start = _monster.getQuantizedLocation();
					_monster.m_PathcirclesPlayer = getPathFortarget(start, goal);
				}
				return true;
			}
		}
	};

	// Chase down
	class ChaseDown : public Task {
	public:
		bool RunTask() override
		{
			if (Is_CloseToPlayer(40))
			{
				return false;
			}
			else {
				// keep chnaging monster path circle by calculation pos
				Location goal = _player.getQuantizedLocation();
				Location start = _monster.getQuantizedLocation();
				_monster.m_PathcirclesPlayer = getPathFortarget(start, goal);
				_monster.m_CurTarget = 0;
				return true;
			}
		}
	};

	// Kill
	class InstantKill : public Task {
	public:
		bool RunTask() override
		{
			// draw square box and kill
			if (rand() % 5 < 3)
			{
				// add traps
				Location posMonst = _monster.getQuantizedLocation();
				// reset player
				int x = posMonst.x;
				int y  = posMonst.y;
				_Traps = { {x + 1, y}, {x, y + 1 },
				{x - 1, y}, {x, y - 1},
				{x + 1,y + 1}, {x + 1,y - 1},
				{x - 1,y - 1}, {x - 1,y + 1} };
				_IsTrapped = true;

				return true;
			}
			else
			{
				return false;
			}
		}
	};

	class BoxKill : public Task {
	public:
		bool RunTask() override
		{
			// add traps
			Location posMonst = _monster.getQuantizedLocation();
			// reset player
			int x = posMonst.x;
			int y = posMonst.y;
			_Traps = { {x + 2, y}, {x, y + 2 },
			{x - 2, y}, {x, y - 2},
			{x + 2,y + 2}, {x + 2,y - 2},
			{x - 2,y - 2}, {x - 2,y + 2} };
			_IsTrapped = true;

			return true;

		}
	};


	// ******************************************************

	// select Decion Making algo type
	DecisionAlgoType s_AlgoType = DecisionAlgoType::MonsterChase;

	// for grid
	GraphWithWeights s_Grid(1, 1);

	ofVec2f linePos1(s_MarginLeftX, s_MarginTopY);
	ofVec2f linePosVert(s_MarginLeftX, s_MarginTopY + s_Width);
	ofVec2f linePosHor(s_MarginLeftX + s_Width, s_MarginTopY);

	std::vector<Location> s_Obstacles;
	Location s_GoalA, s_GoalB;
	Location s_Start({ 1,4 });

	
	physics::SteeringOutput steer;

	// Decision Making variables
	float s_MaxVel = 2; // 3
	float s_MaxAcceleration = 6; // 12
	int s_ClickCounter = 0;

	// monster chase
	Sequencer * s_mainRoot;
}

// ======= ============ ============

float getDistancebetweenPlayandMonster()
{
	Location posPlayer = getQuanizedLocation(
		_player.m_Character.mChar.mPosition.x,
		_player.m_Character.mChar.mPosition.y);

	Location posMonst = getQuanizedLocation(
		_monster.m_Character.mChar.mPosition.x,
		_monster.m_Character.mChar.mPosition.y);

	return getStraightDistance(posPlayer, posMonst) * s_CellSize;

}

bool Is_CloseToPlayer(float dist)
{
	float d = getDistancebetweenPlayandMonster();
	if (d < dist)
		return true;
	return false;
}

// ======= ============ ============


// ====================================

void CreateBehaviorTree()
{
	// create tasks
	RoamAway* t_roam = new RoamAway();
	ChaseDown* t_chase = new ChaseDown();

	UntilFail* ufail_1 = new UntilFail(); // roam until 
	ufail_1->m_Task = t_roam;

	UntilFail* ufail_2 = new UntilFail(); // chase until close
	ufail_2->m_Task = t_chase;

	InstantKill * t_kill = new InstantKill();
	BoxKill * t_Box = new BoxKill();

	Selector* KillSelector = new Selector();
	KillSelector->m_Tasks.push_back(t_kill);
	KillSelector->m_Tasks.push_back(t_Box);

	Sequencer * tmp = new Sequencer(); // A
	tmp->m_Tasks.push_back(ufail_1); // 1
	tmp->m_Tasks.push_back(ufail_2); // 2
	tmp->m_Tasks.push_back(KillSelector); // 3

	s_mainRoot = tmp;
}

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
	/*	  L{5, 3}, L{15, 15}, L{15, 16},
	  L{15, 7}, L{16, 12}, L{16, 13},
	  L{16, 1}, L{16, 17},
	  L{7, 19}, L{17, 15}*/
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
		_player.m_Character.mChar.mPosition.x,
		_player.m_Character.mChar.mPosition.y);
	Location goal = s_GoalA;
	std::unordered_map<Location, Location> came_from;
	std::unordered_map<Location, double> cost_so_far;
	if(s_AlgoType == AlgoType::DjKstra)
		Dijkstra_Search(s_Grid, start, goal, came_from, cost_so_far);
	else
		a_star_search(s_Grid, start, goal, came_from, cost_so_far);
		
	std::vector<Location> path = reconstruct_path(start, goal, came_from);
	_player.m_PathcirclesPlayer = path;
}

std::vector<Location> getRandomPathFortarget(Location start)
{
	int x = rand() % 20; // generate random target
	int y = rand() % 20;
	Location goal(x, y);
	return getPathFortarget(start, goal);
}

std::vector<Location> getPathFortarget(Location start, Location goal)
{
	std::unordered_map<Location, Location> came_from;
	std::unordered_map<Location, double> cost_so_far;
	a_star_search(s_Grid, start, goal, came_from, cost_so_far);

	std::vector<Location> path = reconstruct_path(start, goal, came_from);
	return path;
}


// decision making
void ofApp::RunDecisionTree()
{
	// running two algos
	Location start = getQuanizedLocation(
		_player.m_Character.mChar.mPosition.x,
		_player.m_Character.mChar.mPosition.y);
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
	_player.m_Character.mChar.mMaxVel = s_MaxVel;
	_player.m_Character.mMaxAccel = s_MaxAcceleration;

	_player.m_PathcirclesPlayer = (sizeA < sizeB) ? pathA : pathB;
	 
}


// Helpers ==============

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
		_player.m_Character = AI::KinemSeek(physics::Kinematic(), physics::Kinematic(), 8.0f);
		_player.m_Character.mChar.mMaxVel = s_MaxVel;
		_player.m_Character.mMaxAccel = s_MaxAcceleration;
		_player.m_Character.mMaxSpeed = 6;
		_player.m_Character.mSlowRadArrive = 25;
		_player.m_Character.mTargetRadArrive = 10;
		_player.m_Character.mTimeTotargetArrive = 0.4f;
		_player.UpdateTarget(ofGetLastFrameTime());
		_monster.m_Character = _player.m_Character;
		_monster.m_PathcirclesPlayer = _player.m_PathcirclesPlayer;
		_monster.UpdateTarget(ofGetLastFrameTime());
		_monster.m_Character.mChar.mPosition = ofVec2f(600, 600);
		break;

	case DecisionAlgoType::MonsterChase:
		s_Grid = createGridGraph();
		ExecuteGridExample();
		_player.m_Character = AI::KinemSeek(physics::Kinematic(), physics::Kinematic(), 8.0f);
		_player.m_Character.mChar.mMaxVel = s_MaxVel;
		_player.m_Character.mMaxAccel = s_MaxAcceleration;
		_player.m_Character.mMaxSpeed = 6;
		_player.m_Character.mSlowRadArrive = 25;
		_player.m_Character.mTargetRadArrive = 10;
		_player.m_Character.mTimeTotargetArrive = 0.4f;
		_player.UpdateTarget(ofGetLastFrameTime());
		_monster.m_Character = _player.m_Character;
		_monster.m_Character.mChar.mMaxVel = 4;
		_monster.m_Character.mMaxAccel = 11;
		_monster.m_PathcirclesPlayer = _player.m_PathcirclesPlayer;
		_monster.UpdateTarget(ofGetLastFrameTime());
		_monster.m_Character.mChar.mPosition = ofVec2f(600, 600);
		_player.m_Character.mChar.mPosition = ofVec2f(100, 100);
		CreateBehaviorTree();
		break;
	default:
		break;
	}

}

//--------------------------------------------------------------
void ofApp::update(){
	switch (s_AlgoType)
	{

	case DecisionAlgoType::DecisionTree:

		_player.UpdateTarget(ofGetLastFrameTime());
		//_monster.UpdateTarget(ofGetLastFrameTime());
		break;

	case DecisionAlgoType::MonsterChase:
		s_mainRoot->RunTask();
		_player.UpdateTarget(ofGetLastFrameTime());
		_monster.UpdateTarget(ofGetLastFrameTime());

		if (_player.m_IsTargetReached && !_IsTrapped)
		{
			_player.Reset(); // reset
			int x = rand() % 20; // generate random target
			int y = rand() % 20;
			Location start = _player.getQuantizedLocation();
			Location goal(x, y);
			_player.m_PathcirclesPlayer = getPathFortarget(start, goal);
		}
		if (_IsTrapped)
			_player.m_Character.mChar.mVelocity = ofVec2f(0,0);
		break;

	default:
		break;
	}
	float d = getDistancebetweenPlayandMonster();
}

//--------------------------------------------------------------
void ofApp::draw(){
	// Grid Stuff
	//if (s_AlgoType == DecisionAlgoType::DecisionTree)
	{
		ofFill();
		ofSetColor(200, 0, 0);
		DrawGrid();
		
		_player.DrawPath(); // draw path
		_monster.DrawPath();

		auto size = _player.m_PathcirclesPlayer.size();
		ofSetColor(0, 0, 250); // Blue Goals
		DrawCircleInCell(s_GoalA.x, s_GoalA.y );
		DrawCircleInCell(s_GoalB.x, s_GoalB.y);

		ofSetColor(240, 0, 0);
		for (auto s : s_Grid.forests) // draw forests
			DrawCircleInCell(s.x, s.y);

		// follow stuff
		ofSetColor(250, 0, 150);
		ofNoFill();
		ofFill();
		ofDrawLine(_player.m_Character.mChar.mPosition,
			_player.m_Character.mChar.mPosition + 10 * steer.mLinear); // acceleration line

		drawBoid(_player.m_Character.mChar.mPosition,
			_player.m_Character.mChar.mOrientation,
		
			ofColor(150, 180, 150));
		drawBoid(_monster.m_Character.mChar.mPosition,
			_monster.m_Character.mChar.mOrientation,
			ofColor(250, 0, 150));

		// draw close radius
		ofNoFill();
		ofDrawCircle(_monster.m_Character.mChar.mPosition, 500);

		ofFill();

		// DrawTrap
		for (auto l : _Traps)
		{
			ofColor(153, 0, 76);
			DrawCircleInCell(l.x, l.y);
		}
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

// reset
void resetBehavTree()
{
	_IsTrapped = false;
	_Traps.clear();
	_player.m_Character.mChar.mPosition = ofVec2f(rand()%100, rand()%100);
	_monster.m_Character.mChar.mPosition = ofVec2f(600, 600);
	_player.Reset();
	_monster.Reset();
	_player.m_PathcirclesPlayer = getRandomPathFortarget(_player.getQuantizedLocation());
	_monster.m_PathcirclesPlayer = getRandomPathFortarget(_monster.getQuantizedLocation());
	CreateBehaviorTree();
}

//--------------------------------------------------------
// MOUSE PRESSED
//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{
	auto loc = getQuanizedLocation(x, y);
	
	if (s_AlgoType == DecisionAlgoType::DecisionTree)
	{
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
				_player.Reset(); // reset
				//s_circles.push_back(loc);
			}
		}
	}
	if (s_AlgoType == DecisionAlgoType::MonsterChase)
	{
		resetBehavTree();
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
