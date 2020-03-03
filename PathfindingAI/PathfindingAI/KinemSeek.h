#pragma once
#include "Kinematic.h"

namespace AI {

	class KinemSeek
	{
	public:

		KinemSeek() {};

		KinemSeek(physics::Kinematic c, physics::Kinematic tar, float maxSp)
			: mCharacter(c), mTarget(tar), mMaxSpeed(maxSp){}

		// returns steering based on player and target
		physics::SteeringOutput getSteering();
		// Dynamic arrive
		physics::SteeringOutput getSteeringForArrival();
		// Wander
		physics::SteeringOutput getSteeringForWandering();
		
		physics::Kinematic mCharacter;
		physics::Kinematic mTarget;
		bool mSlowRadReached = false;
		float mMaxSpeed;
		float mMaxAccel;
		float mMaxRotat;
		// for arrival
		float mTargetRadArrive;
		float mSlowRadArrive;
		float mTimeTotargetArrive;
		// for align
		float mTargetRadAlign;
		float mSlowRadAlign;
		float mTimeTotargetAlign;
	};
}

