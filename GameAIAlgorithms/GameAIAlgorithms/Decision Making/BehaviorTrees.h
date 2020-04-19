#pragma once
#include <vector>

// Behavior Trees
	using namespace std;

	class Task {
	public:
		virtual bool RunTask() { return false; };
	};

	// succeeds if one of the children succeeds
	class Selector: public Task {
	public:
		// goes through children and return true if one of the children runs
		bool RunTask() override
		{
			for (Task* t : m_Tasks)
			{
				if (t-> RunTask())
					return true;
			}
			return false;
		}


	public:
		vector<Task*> m_Tasks;
	};

	// succeeds if all children succeeds
	class Sequencer: public Task {
	public:
		// goes through children and return true all children runs
		bool RunTask() override
		{
			for (Task* t : m_Tasks)
			{
				if (!t->RunTask())
				{
					return false;
				}
				else
				{
					m_Tasks.erase(m_Tasks.begin());
					return false;
				}
			}
			return true;
		}

	public:
		vector<Task*> m_Tasks;
	};


	// Modify child behavior
	class Decorator {

	public:
		virtual bool RunModified() { return false; };
	};

	// invert the result
	class Inverter {
	public:
		bool Run()
		{
			if (m_Task->RunTask())
				return false;
			return true;
		}
	private:
		Task* m_Task;
	};

	// example of decorator
	class UntilFail : public Decorator, public Task {
	
	public:
		bool RunTask() override
		{
			/*while (true)
			{
				if (!m_Task->RunTask())
					break;
			}
			return true;*/

			if (m_Task->RunTask())
				return false;
			return true;
		}

	public:
		Task* m_Task;

	};

