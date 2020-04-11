#pragma once

class Decision{

public:
	


	// run tests and retunr the decision
	virtual Decision* getBranch() { return nullptr; }

	Decision* makeADecision()
	{
		Decision* b = getBranch();
		return (b->m_HasAction) ? b : b->makeADecision();
	}

public:
	bool m_HasAction = false;
	float mVel = 0;
	float mAccel = 0;
	Decision* m_BranchTrue = nullptr;
	Decision* m_BranchFalse = nullptr;
};