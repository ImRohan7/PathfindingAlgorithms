#pragma once
#include <iostream>
#include "DataStructures.h"

/* Includes small helper tools for pathfinding such as 
>> Quantization
>> Localization
*/

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
