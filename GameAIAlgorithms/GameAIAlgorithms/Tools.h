#pragma once
#include <iostream>
#include "DataStructures.h"

/* Includes small helper tools for pathfinding such as 
>> Quantization
>> Localization
*/

enum DecisionAlgoType
{
	DecisionTree,
	BehavTree,
};

// the version to run
enum AlgoVersion{
	CampusMap,
	BigDataMap,
	InteractiveGrid,
};

// the type of algo to run
enum AlgoType {
	DjKstra,
	AStar,
};

namespace {
	
	// intended screen width for drawing measurement
	float s_Width = 740; 

	float s_boxPerLine = 20;
	float s_MarginLeftX = 10;
	float s_MarginTopY = 10;
	float s_CellSize = 37; // cell height and width

}
// TILE Based conversion

// Qauntization
// Parameters: pass screen coordinates
Location getQuanizedLocation( float iScreenX, float  iScreenY)
{
	int xi = (iScreenX - s_MarginLeftX) / s_CellSize;
	int yi = (iScreenY - s_MarginTopY) / s_CellSize;
	
	return Location(xi,yi);
}

// Localization
std::pair<int, int> getLocalizedOnScreenPosition(Location iLoc)
{
	std::pair<int, int> result(1,1);
	result.first = (iLoc.x * s_CellSize) + s_MarginLeftX;
	result.second = (iLoc.y * s_CellSize) + s_MarginTopY;

	return result;
}

// get Absolute vec for on screen
ofVec2f getAbsoluteObjectPosition(Location iLoc)
{
	ofVec2f result(1, 1);
	result.x = (iLoc.x * s_CellSize) + s_MarginLeftX;
	result.y = (iLoc.y * s_CellSize) + s_MarginTopY;
	result += s_CellSize / 2;
	return result;
}

double getStraightDistance(Location a, Location b)
{
	// we use pythagoras theorem z^2 = x^2 + y^2;
	double x = std::abs(a.x - b.x);
	double y = std::abs(a.y - b.y);
	x *= x;
	y *= y;
	double z = sqrt(x + y);
	return z;
}

// Draw circle in cell
void DrawCircle_InCell(int x, int y)
{
	auto pos = getLocalizedOnScreenPosition({ x,y });
	ofVec2f po(pos.first, pos.second);
	ofDrawCircle(po + s_CellSize / 2, 15.0f);
}