#pragma once
#include "Kinematic.h"
#include <vector>

#define MaxRotation 8.0f // max rotation velocity
#define MaxAccelAngular 8.5f

using namespace std;
namespace AISystem {
	enum Algo
	{
		Basic_Kinematic,
		SeekArrive,
		SeekArrive2,
		WanderSteering,
		Flocking
	};

	// Calculate Center of Mass
	void calcCom(
		physics::Kinematic leader, 
		vector<physics::Kinematic> foll
	);

	// Calculate Center of Velocity
	void calcCovel(
		physics::Kinematic leader, 
		vector<physics::Kinematic> foll
	);

	// Align
	 physics::SteeringOutput getSteeringFor_Align(
		float iTarOri,
		float iCharOri, 
		float iSlowRadi, 
		float iTarRadi, 
		float iTimeToTarget
	);

	// Flocking
	physics::SteeringOutput getSteeringForFlocking(
		physics::Kinematic leader,
		vector<physics::Kinematic> foll,
		int i
	);



	/*
	switch (sAlgo)
	{
	case AISystem::Algo::Kinematic:

		break;
	case AISystem::Algo::SeekArrive:
	case AISystem::Algo::SeekArrive2:

		break;
	case AISystem::Algo::WanderSteering:

		break;
	case AISystem::Algo::Flocking:

		break;

	default:
		std::cout << "Unknown case passed";
		break;
	}
	*/
}
