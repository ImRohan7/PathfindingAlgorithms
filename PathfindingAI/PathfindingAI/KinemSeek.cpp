#include "KinemSeek.h"
#include <random>

#define MaxRotation 5.0f // max rotation velocity

// for no arrival
physics::SteeringOutput AI::KinemSeek::getSteering()
{
	physics::SteeringOutput steering;

	ofVec2f dir = (mTarget.mPosition - mCharacter.mPosition).normalize(); // get vel dir
	steering.mLinear = dir * mMaxSpeed;
	
	return steering;
}


// with dynamic arrive
physics::SteeringOutput AI::KinemSeek::getSteeringForArrival()
{
	physics::SteeringOutput steering;
	
	ofVec2f dir = mTarget.mPosition - mCharacter.mPosition; // get vel dir
	float distance = dir.length();
	float targetSpeed = 0.0f;
	ofVec2f targetVel;
	// if we already reached
	if (distance < mTargetRadArrive)
	{
		mSlowRadReached = true;
		return steering;
	}
	// if outside slow rad then we go max velocity
	if (distance > mSlowRadArrive)
		targetSpeed = mMaxSpeed;

	// else we scale the speed with the distance remaining
	else
		targetSpeed = mMaxSpeed * (distance / mSlowRadArrive);

	//
	targetVel = dir.normalize();
	targetVel *= targetSpeed;

	// get acceleration dir
	steering.mLinear = targetVel - mCharacter.mVelocity;
	//steering.mLinear = targetVel;
	steering.mLinear /= mTimeTotargetArrive;

	// clamp acceleration
	if (steering.mLinear.length() > mMaxAccel)
	{
		steering.mLinear.normalize();
		steering.mLinear *= mMaxAccel;
	}
	steering.mAngular = 0;
	return steering;
}

// wandering
physics::SteeringOutput AI::KinemSeek::getSteeringForWandering()
{
	physics::SteeringOutput steering;
	float x = cos(mCharacter.mOrientation);
	float y = sin(mCharacter.mOrientation);
	steering.mLinear = mMaxSpeed * ofVec2f(x, y);

	float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	steering.mAngular = (rand()%2 == 0) ? r : -r;
	steering.mAngular *= mMaxRotat;

	return steering;
}