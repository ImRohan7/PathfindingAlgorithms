#pragma once
#include <vector>

// Behavior Trees
	using namespace std;

	class Task {
	public:
		virtual bool RunTask() { return false; };
	};

	// succeeds if one of the children succeeds
	class Selector {

		// goes through children and return true if one of the children runs
		bool Run()
		{
			for (Task* t : m_Tasks)
			{
				if (t-> RunTask())
					return true;
			}
			return false;
		}


	private:

		vector<Task*> m_Tasks;
	};

	// succeeds if all children succeeds
	class Sequence {

		// goes through children and return true all children runs
		bool Run()
		{
			for (Task* t : m_Tasks)
			{
				if (!t->RunTask())
					return false;
			}
			return false;
		}


	private:

		vector<Task*> m_Tasks;
	};


	// Modify child behavior
	class Decorator {

	public:
		virtual bool RunModified();
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
	class UnitlFail : public Decorator, public Task {
	
	public:
		bool RunModified() override
		{
			while (true)
			{
				if (!m_Task->RunTask())
					break;
			}
		}

	private:
		Task* m_Task;

	};

