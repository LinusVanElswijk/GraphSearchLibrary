#ifndef GRID_MAP_PROBLEM_HPP
#define GRID_MAP_PROBLEM_HPP

#include "Problem.hpp"

extern "C++"
{
namespace searchAlgorithms
{

//////////////////////////////////////////////////////////////////////////////
// class GridMapProblem definition													//
//////////////////////////////////////////////////////////////////////////////

	/**
	 * Abstract class to represent search problems.
	 * @tparam StateType the type of states of the problem
	 * @tparam ActionType the type of the actions that can be performed on nodes to get to the next node.
	 * @tparam CostType the type of the cost of actions
	 */
	template <typename StateType, typename ActionType, typename CostType, typename AngleType>
    class GridMapProblem: public Problem<StateType, ActionType, CostType>
	{
		public:
			/**
			 * Structure to represent (action, result) tuples
			 */
			typedef typename Problem<StateType, ActionType, CostType>::ActionAndResult ActionAndResult;

			virtual GridMapProblem<StateType, ActionType, CostType, AngleType>& operator= (const GridMapProblem<StateType, ActionType, CostType, AngleType>& otherProblem)
			{
				Problem<StateType, ActionType, CostType>::operator=(otherProblem);
				return *this;
			}

			/**
			 * Returns the list of successors of a state
			 * @param[in] state the state for which the successor will be given
			 * @return the list of successors
			 */
			virtual std::list<ActionAndResult> getSuccessors(const StateType& state) const = 0;

			/**
			 * Tests wheter a state is a goal state or not.
			 * @param[in] state the state to test
			 * @return true if the state is a goal state,
			 *		   false otherwise
			 */
			virtual bool isGoal(const StateType& state) const = 0;

			/**
			 * Gives the cost of doing an action in certain state
			 * @param[in] state in which the action is performed
			 * @param[in] the action
			 * @return the cost of the action, when performed in the state
			 */
			virtual CostType actionCost(const StateType& state, const ActionType& action) const = 0;

			virtual bool haveLineOfSight(const StateType& origin, const StateType& destination) const = 0;

			virtual ActionType getAction(const StateType& origin, const StateType& destination) const = 0;

			virtual AngleType angle( const StateType& from1, const StateType& to1,
					                 const StateType& from2, const StateType& to2
					               ) const = 0;

			virtual std::list<StateType> getCorners(const StateType& state) const = 0;

			virtual std::list<StateType> getNeighbors(const StateType& state) const = 0;

			virtual bool isCornerOfBlockedCell(const StateType& state) const = 0;

			virtual bool isFree(const StateType& state) const = 0;

			//virtual bool getNeighbouringBlockedCells() = 0;

		protected:
			inline GridMapProblem(StateType initialState)
			:Problem<StateType, ActionType, CostType>(initialState)
			{}
	};
}
}
#endif /* GRID_MAP_PROBLEM_HPP */
