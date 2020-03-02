#pragma once
#include <iostream>
#include "DataStructures.h"

/* Includes small helper tools for pathfinding such as 
>> Quantization
>> Localization
*/
namespace {

	float s_MarginLeftX = 10;
	float s_MarginTopY = 10;
	float s_CellSize = 70; // cell height and width

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
