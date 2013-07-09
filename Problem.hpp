#ifndef SEARCH_ALGORITHMS_PROBLEM_HPP
#define SEARCH_ALGORITHMS_PROBLEM_HPP

#include <list>
#include <memory>

extern "C++"
{
namespace searchAlgorithms
{

//////////////////////////////////////////////////////////////////////////////
// class Problem definition													//
//////////////////////////////////////////////////////////////////////////////

	/**
	 * Abstract class to represent search problems.
	 * @tparam StateType the type of states of the problem
	 * @tparam ActionType the type of the actions that can be performed on nodes to get to the next node.
	 * @tparam CostType the type of the cost of actions
	 */
    template <typename StateType, typename ActionType, typename CostType>
    class Problem
	{
		public:
            typedef Problem Type;
            typedef StateType  State;
            typedef ActionType Action;
            typedef CostType   Cost;

			/**
			 * Structure to represent (action, result) tuples
			 */
            struct ActionAndResult;

            /**
             * Destructor for a Problem
             */
            virtual ~Problem();

			/**
			 * Returns the list of successors of a state
			 * @param[in] state the state for which the successor will be given
			 * @return the list of successors
             *
             */
            virtual std::list<ActionAndResult> getSuccessors(const State& state) const = 0;

			/**
			 * Tests wheter a state is a goal state or not.
			 * @param[in] state the state to test
			 * @return true if the state is a goal state,
			 *		   false otherwise
             */
            bool isGoal(const State& state) const;

			/**
			 * Gives the cost of doing an action in certain state
			 * @param[in] state in which the action is performed
			 * @param[in] the action
			 * @return the cost of the action, when performed in the state
             */
            virtual Cost actionCost(const State& state, const Action& action) const = 0;

			/**
			 * Returns the initial state of the problem
			 * @return the initial state of the problem
             */
            const State& initialState() const;

			/**
			 * Returns the goal state of the problem
			 * @return the goal state of the problem
             */
            const State& goalState() const;

		protected:
            Problem(const State& initialState, const State& goalState)
            :initial(initialState),
             goal(goalState)
			{}

            const State &initial, &goal;
    };

//////////////////////////////////////////////////////////////////////////////
// struct ActionAndResult definition										//
//////////////////////////////////////////////////////////////////////////////

	template <typename StateType, typename ActionType, typename CostType>
	struct Problem<StateType, ActionType, CostType>::ActionAndResult
	{
		public:
            typedef Problem<StateType, ActionType, CostType>::Action Action;
            typedef Problem<StateType, ActionType, CostType>::State  State;

			/**
			 * Constructor
			 * @param[in] value of action
			 * @param[in] value of result
             */
            ActionAndResult(const Action& action, const State& result)
			:action(action),
			 result(result)
			{}

			/**
			 * The action
             */
            const Action &action;

			/**
			 * The result of the action
             */
            const State &result;
	};

//////////////////////////////////////////////////////////////////////////////
// member function implementation											//
//////////////////////////////////////////////////////////////////////////////
// public:

	template <typename StateType, typename ActionType, typename CostType>
	Problem<StateType, ActionType, CostType>::~Problem()
	{ }

    /*
	template <typename StateType, typename ActionType, typename CostType>
	Problem<StateType, ActionType, CostType>& Problem<StateType, ActionType, CostType>::operator= (const Problem<StateType, ActionType, CostType>& sourceProblem)
	{	
		this->initial = sourceProblem.initial;
		this->goal = sourceProblem.goal;
		return *this;
    }*/

	template <typename StateType, typename ActionType, typename CostType>
    const typename Problem<StateType, ActionType, CostType>::State& Problem<StateType, ActionType, CostType>::initialState() const
	{	return initial;
	}
	
	template <typename StateType, typename ActionType, typename CostType>
    const typename Problem<StateType, ActionType, CostType>::State& Problem<StateType, ActionType, CostType>::goalState() const
	{	return goal;
	}

	template <typename StateType, typename ActionType, typename CostType>
    bool Problem<StateType, ActionType, CostType>::isGoal(const StateType& state) const
    {	return state == goal;
	}

}
}

#endif
