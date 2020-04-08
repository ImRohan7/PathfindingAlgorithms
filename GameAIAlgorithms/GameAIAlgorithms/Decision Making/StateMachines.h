#pragma once
#include <vector>

// SM: State Machines
namespace SM {


	class Action {

		virtual void Perform();

	};

	// += for actions
	std::vector<Action*> operator+=(std::vector<Action*> a, std::vector<Action*> b)
	{
		for (auto val : b)
		{
			a.push_back(val);
		}
		return a;
	}

	class State {
	
	public:

		// Getters
		inline std::vector<Action*> getActions() { return m_Actions; }
		inline std::vector<Action*> getEntryActions() { return m_EntryActions; }
		inline std::vector<Action*> getExitActions() { m_ExitActions; }
		inline std::vector<Transition*> getTransitions() { m_Transitions; }
		
	private:
		std::vector<Action*> m_Actions;
		std::vector<Action*> m_EntryActions;
		std::vector<Action*> m_ExitActions;
		std::vector<Transition*> m_Transitions;
	};

	class Condition {
	
	public:

		bool Test()
		{
			return false;
		}

	};

	class Transition {
	
	public:

		bool IsTriggered() { return m_Condition->Test(); }
		
		// Getters
		inline State* getTargetState() { return m_TargetState;  }
		inline std::vector<Action*>  getActions() { return m_Actions; }
			

	private:
		std::vector<Action*> m_Actions;
		State* m_TargetState;
		Condition* m_Condition;
	};


	




	class StateMachine {

	public:

		// update
		std::vector<Action*> update();

	private:
		State* m_InitialState;
		State* m_CurrentState;
	};
}