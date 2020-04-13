#include "PathFollower.h"
#include "AISystem.h"
#include "Tools.h"

// update target for follow
void Follower::UpdateTarget(float lastFrameTime)
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



void Follower::Reset()
{
	m_CurTarget = 0;

	m_Character.mChar.mVelocity = ofVec2f(0, 0);
	//	seek.mCharacter.mPosition = getAbsoluteObjectPosition(s_Start);
	m_Character.mTarget.mPosition = m_Character.mChar.mPosition;
}