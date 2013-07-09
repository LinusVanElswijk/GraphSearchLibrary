// Linus van Elswijk 0710261

#ifndef SEARCH_ALGORITHMS_HEURISTIC_FUNCTION_HPP
#define SEARCH_ALGORITHMS_HEURISTIC_FUNCTION_HPP

extern "C++"
{
namespace searchAlgorithms
{

//////////////////////////////////////////////////////////////////////////////
// class HeuristicFunction definition										//
//////////////////////////////////////////////////////////////////////////////

	/**
	 * Abstract class to represent heuristic functions (as objects)
	 * @tparam the type of the states the heuristic function works on
	 * @tparam the type of the estimated cost
	 */
	template <typename StateType, typename CostType>
	class HeuristicFunction
	{
		public:

            /**
             * Destructor for HeuristicFunction
             */
            virtual ~HeuristicFunction()
            {
            }
			
			/**
			 * Creates a dynamic allocated copy of the heuristic function.
			 * Calls the copy constructor of the runtime class.
			 */
			virtual HeuristicFunction* clone() const = 0;
			
			/**
			 * Performs the heuristic function on a state
			 * @param[in] the state
			 * @return the estimated cost to reach the goal from the state
			 */
            virtual CostType operator() (const StateType& currentState) const = 0;
    };

}
}
#endif
