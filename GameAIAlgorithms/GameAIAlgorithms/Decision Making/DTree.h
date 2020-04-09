#pragma once

class Decision{

public:
	
	// run tests and retunr the decision
	virtual Decision* getBranch();

	Decision* makeADecision()
	{
		Decision* b = getBranch();
		return (b->m_HasAction) ? b : b->makeADecision();
	}

public:
	bool m_HasAction = false;
	float mSpeed;
	float mAccel;
	Decision* m_BranchTrue;
	Decision* m_BranchFalse;
};