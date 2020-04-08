#include "StateMachines.h"

namespace SM {


	std::vector<Action*> StateMachine::update()
	{
		Transition* triggered = nullptr;

		for (Transition* trans : m_CurrentState->getTransitions())
		{
			if (trans->IsTriggered())
			{
				triggered = trans;
				break;
			}
		}

		// if transition is triggered then return actions
		if (triggered)
		{
			// get target state
			State* targetState = triggered->getTargetState();

			// exit actions of current state
			std::vector<Action*> actions;
			actions = m_CurrentState->getExitActions();
			// add transition actions
			actions += triggered->getActions();
			// add new state entry actions
			actions += targetState->getActions();

			// update the cur state
			m_CurrentState = targetState;
			return actions;
		}
		else
		{
			return m_CurrentState->getActions();
		}

	}

}