#include "DataStructures.h"

bool operator == (Location a, Location b) {
	return a.x == b.x && a.y == b.y;
}