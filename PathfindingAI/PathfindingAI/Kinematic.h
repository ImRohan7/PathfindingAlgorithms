#pragma once
#include "ofMath.h"
#include "ofVectorMath.h"


namespace physics {

	/* Steering behaviors operate with these kinematic data. They return accelerations that will
	  change the velocities of a character in order to move them around the level. Their output is a set
		of accelerations:	*/

	struct SteeringOutput {

	public:
		ofVec2f mLinear = ofVec2f(0.0f, 0.0f); // linear acceleration
		float mAngular = 0.0f;	// angluar acceleration
	};

	struct Kinematic
	{

		void update(SteeringOutput iSteering, float iElapsedTime);
		static float getNewOrientation(ofVec2f ivel, float iCurOrientation);
		void updateOrientation(SteeringOutput st); // based on velocity

		Kinematic(ofVec2f pos) : mPosition(pos){}
		Kinematic() {};

	public:
		ofVec2f mPosition;
		ofVec2f mVelocity;
		float mOrientation = 0.0f;
		float mRotaionvel = 0.0f; // rot velocity
		float mWeight;
		float mSepRadius;
	};

	
}

