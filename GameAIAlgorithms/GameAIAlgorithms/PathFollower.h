#pragma once
#include "KinemSeek.h"
#include "DataStructures.h"

/* Assist while following the guided path circles on grid map
>> Updates the target as soon as it reches nearest radius

*/


class Follower {

public:

	void initliaze();
	void UpdateTarget();
	void Reset();

public:
	AI::KinemSeek m_Character;
	int m_CurTarget; // index of current target
	std::vector<Location> m_PathcirclesPlayer; // path

};