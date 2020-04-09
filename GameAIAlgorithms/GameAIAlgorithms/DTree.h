#pragma once


struct TreeNode {

public:
	float val;
	TreeNode* True;
	TreeNode* False;


};

class Decision{
	// or I can store an action pointer here instead of storing it as a whole
public:
	bool IsAction = false;
	float val = 0.0f;
	Decision* True;
	Decision* False;
	float mintest;
	float Maxtest;

	float getCharacterVelocity();

	// run tests and retunr the decision
	Decision* getBranch()
	{
		// srite the logic
		return False;
	}

	Decision* makeADecision()
	{
		Decision* b = getBranch();
		return (b->IsAction) ? b : b->makeADecision();
	}
};