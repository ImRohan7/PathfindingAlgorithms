#pragma once
#include "DTree.h"

// This header contains definitions for custom decisions

class ChooseAlgo : public Decision {
public:
	
	ChooseAlgo(int a, int b) : StepsTargetA(a), StepsTargetB(b){}

	// run tests and retun the decision
	Decision* getBranch() override
	{
		// write the logic
		if (StepsTargetA < StepsTargetB)
			return m_BranchTrue;
		return m_BranchFalse;
	}


public:
	int StepsTargetA = 0;
	int StepsTargetB = 0;
};

class ChooseSpeed : public Decision {

public:
	ChooseSpeed(int a, int b) : 
		LinearDistA(a), LinearDistB(b) {}

	// run tests and retun the decision
	Decision* getBranch() override
	{
		// write the logic
		if (LinearDistA < LinearDistB)
			return m_BranchTrue;
		return m_BranchFalse;
	}

public:
	float LinearDistA;
	float LinearDistB;
};