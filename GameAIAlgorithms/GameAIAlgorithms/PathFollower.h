#pragma once
#include "KinemSeek.h"
#include "DataStructures.h"
#include "Tools.h"

/* Assist while following the guided path circles on grid map
>> Updates the target as soon as it reches nearest radius

*/

class Follower {

public:

	void UpdateTarget(float lastFrameTime)
	{
		if (m_Character.mSlowRadReached)
		{
			m_Character.mSlowRadReached = false;
			// change target position
			m_CurTarget++;
			if (m_CurTarget >= m_PathcirclesPlayer.size())
				m_CurTarget--;
			auto newTarget = getAbsoluteObjectPosition(m_PathcirclesPlayer[m_CurTarget]);
			m_Character.mTarget.mPosition = newTarget;
		}
		physics::SteeringOutput steer;

		//  get steering and update 
		steer = m_Character.getSteeringForArrival();
		float tOr = atan2(m_Character.mChar.mVelocity.y,
			m_Character.mChar.mVelocity.x);
		steer.mAngular = AISystem::getSteeringFor_Align(
			tOr, m_Character.mChar.mOrientation,
			1.5, 0.3f, 2.5).mAngular;
		m_Character.mChar.update(steer, lastFrameTime); // update 


	}
	
	void DrawPath()
	{
		ofSetColor(83, 228, 250); // Yellow circles path
		for (auto s : m_PathcirclesPlayer)
			DrawCircle_InCell(s.x, s.y);
		
		ofSetColor(10, 223, 60); // Green circles First
		DrawCircle_InCell(m_PathcirclesPlayer[0].x,
			m_PathcirclesPlayer[0].y);
	}

	void Reset()
	{
		m_CurTarget = 0;

		m_Character.mChar.mVelocity = ofVec2f(0, 0);
		//	seek.mCharacter.mPosition = getAbsoluteObjectPosition(s_Start);
		m_Character.mTarget.mPosition = m_Character.mChar.mPosition;

	}
	
public:
	AI::KinemSeek m_Character;
	int m_CurTarget; // index of current target
	std::vector<Location> m_PathcirclesPlayer; // path

};