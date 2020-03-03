#include "AISystem.h"

static ofVec2f sCom; // center of mass
static ofVec2f sComVel; // center of vel

static float sMaxSpeed = 1.0f;

void AISystem::calcCom(
	physics::Kinematic leader, vector<physics::Kinematic> foll)
{
	ofVec2f com;
	float total = 0;
	for (physics::Kinematic k : foll)
	{
		com += (k.mPosition * k.mWeight);
		total += k.mWeight;
	}
	com += (leader.mPosition*leader.mWeight);
	total += leader.mWeight;
	com /= total;
	sCom = com;
}

void AISystem::calcCovel(
	physics::Kinematic leader, vector<physics::Kinematic> foll)
{
	ofVec2f covel;
	float total = 0;
	for (physics::Kinematic k : foll)
	{
		covel += (k.mVelocity * k.mWeight);
		total += k.mWeight;
	}
	covel += (leader.mVelocity*leader.mWeight);
	total += leader.mWeight;
	covel /= total;
	sComVel = covel;
}

// align
physics::SteeringOutput AISystem::getSteeringFor_Align(float iTarOri,
	float iCharOri, float iSlowRadi, float iTarRadi, float iTimeToTarget)
{
	physics::SteeringOutput steering;

	float rot = iTarOri - iCharOri;
	float targetRot = 0.0f;
	// map to tange
	{
		rot = ofRadToDeg(rot);
		rot = (int)rot%360;
		if (rot > 180)
			rot = rot - 360;
		else if (rot < -180)
			rot = 360 - rot;
		rot = ofDegToRad(rot);
	}

	float amount = abs(rot);

	// if already withing target radius
	if (amount < iTarRadi)
		return steering;

	// if outside alow rad accelerate to max
	if (amount > iSlowRadi)
		targetRot = MaxRotation;
	else
		targetRot = MaxRotation * amount / iSlowRadi;

	targetRot *= rot / amount;

	steering.mAngular = targetRot;// -iCharOri;
	steering.mAngular /= iTimeToTarget;

	//clamp accelration
	float absval = abs(steering.mAngular);
	if (absval > MaxAccelAngular)
	{
		steering.mAngular /= absval;
		steering.mAngular *= MaxAccelAngular;
	}

  	steering.mLinear = ofVec2f(0, 0);
	return steering;
}

// retunrs steering individual
physics::SteeringOutput AISystem::getSteeringForFlocking(
	physics::Kinematic leader, vector<physics::Kinematic> foll, int i)
{
	physics::SteeringOutput steer;
	physics::Kinematic chr = foll[i];
	// seek to center of mass + look where going
	// com
	ofVec2f seekVel;
	seekVel = sCom - chr.mPosition;
	seekVel = seekVel.normalize();
	seekVel *= sMaxSpeed;
	// angle
	steer.mAngular = chr.getNewOrientation(seekVel, chr.mOrientation);

	ofVec2f separtVel;
	// separate from others
	for (int j = 0; j < foll.size(); j++)
	{
		if (j != i)
		{
			if (foll[j].mPosition.distance(chr.mPosition) < chr.mSepRadius)
			{
				ofVec2f a = (chr.mPosition - foll[j].mPosition);
				a = a.normalize();
				separtVel += a;
				steer.mLinear = separtVel;
				//return steer;
			}
		}
	}

	// velocity match
	ofVec2f matchVel = sComVel - chr.mVelocity;
	float coef = 10;
	if (seekVel.length() < matchVel.length())
	{
		coef = 5;
	}
	steer.mLinear = seekVel * 20 + separtVel * 100 + matchVel * coef;

	return steer;
}
