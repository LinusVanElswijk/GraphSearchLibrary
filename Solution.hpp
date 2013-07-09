// Linus van Elswijk 0710261

#ifndef SEARCH_ALGORITHMS_SOLUTION_HPP
#define SEARCH_ALGORITHMS_SOLUTION_HPP

#include <iostream>
#include <stdexcept>

#include "Node.hpp"

extern "C++"
{
namespace searchAlgorithms
{

//////////////////////////////////////////////////////////////////////////////
// class Solution definition												//
//////////////////////////////////////////////////////////////////////////////

	/**
	 * A class to represent a solution to a search problem
	 * @tparam ActionType the type of the actions that can be performed on states to get to the next state
	 * @tparam CostType the type of the action costs
	 */
    template <typename ProblemType>
	class Solution
    {
        /*
         * function to put a description of the solution on an output stream
         * @param[inout] outStream stream object, to which the description will be added
         * @param[in] solution the solution that will be added
         * @returns the (modified) stream
         *
        friend std::ostream& operator<< (std::ostream &outStream, const searchAlgorithms::Solution<ProblemType> &solution)
        {
            outStream << "Solution[ Actions = {";

            for(unsigned int i = 0; i < solution.numberOfActions - 1; i++)
            {
                outStream << solution.action[i] << ", ";
            }
            if(solution.numberOfActions != 0)
            {
                outStream << solution.action[solution.numberOfActions - 1];
            }
            outStream << "}, ";
            outStream << "Cost = " << solution.cost << "]";

            return outStream;
        }*/

        public:
            typedef typename ProblemType::State State;
            typedef typename ProblemType::Action Action;
            typedef typename ProblemType::Cost Cost;

			/**
			 * Constructor
			 * @tparam StateType the type of the states in the argument node
			 * @param[in] node that contains a goal state
			 */ // gives errors when defined outside of class body
            Solution(const typename Node<ProblemType>::ConstPtr& node)
            :cost( node->getPathCost() ),
             numberOfActions( node->getDepth() ),
             numberOfStates(node->getDepth() + 1),
             action( new const Action*[numberOfActions] ),
             state( new const State*[numberOfStates] )
			{
                typename Node<ProblemType>::ConstPtr currentNode = node;

                for(unsigned int i = 0; i < numberOfActions; i++)
                {
                    state[numberOfStates - i - 1]   = &currentNode->getState();
                    action[numberOfActions - i - 1] = &currentNode->getActionPerformedOnParent();
                    currentNode = currentNode->getParent();
				}

                state[0] = &currentNode->getState();
			}

            /**
             * Returns the total cost of the solution
             * @return the total cost of the solution
             */
            const Cost& getCost() const;

            /**
             * Returns the number of actions of the solution
             * @return the number of actions
             */
            const unsigned int& getNumberOfActions() const;


            /**
             * Returns the number of states in the solution path
             * @return the number of state
             */
            const unsigned int& getNumberOfStates() const;

            /**
             * Returns an actions of the solution.
             * @param[in] i the number of the action
             * @return the ith action of the solution
             * @throws std::out_of_bounds if i is greater than or equal to the number of actions
             */
            const Action& getAction(const unsigned int& i) const;

            /**
             * Returns a state of the solution path.
             * @param[in] i the number of the action
             * @return the ith state of the solution path
             * @throws std::out_of_bounds if i is greater than or equal to the number of actions
             */
            const State& getState(const unsigned int& i) const;

        private:
            const Cost cost;
            const unsigned int numberOfActions, numberOfStates;
            const Action ** action;
            const State ** state;
	};

//////////////////////////////////////////////////////////////////////////////
// member function implementation											//
//////////////////////////////////////////////////////////////////////////////
// public:

    template <typename ProblemType>
    const typename Solution<ProblemType>::Cost& Solution<ProblemType>::getCost() const
	{	return cost;
	}

    template <typename ProblemType>
    const unsigned int& Solution<ProblemType>::getNumberOfActions() const
	{	return numberOfActions;
	}

    template <typename ProblemType>
    const typename Solution<ProblemType>::Action& Solution<ProblemType>::getAction(const unsigned int& i) const
	{
		if(i >= numberOfActions)
            throw std::out_of_range("No such action in the solution");

        return *action[i];
	}

    template <typename ProblemType>
    const unsigned int& Solution<ProblemType>::getNumberOfStates() const
    {	return numberOfStates;
    }

    template <typename ProblemType>
    const typename Solution<ProblemType>::State& Solution<ProblemType>::getState(const unsigned int& i) const
    {
        if(i >= numberOfStates)
            throw std::out_of_range("No such state in the solution");

        return *state[i];
    }
}
}
#endif
