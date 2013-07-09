#ifndef LINEOFSIGHTFUNCTION_HPP
#define LINEOFSIGHTFUNCTION_HPP

#include <memory>

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
    template <typename ProblemType>
    class LineOfSightFunction
    {
        public:
            typedef typename ProblemType::State  State;
            typedef typename ProblemType::Action Action;
            typedef typename ProblemType::Cost   Cost;

            virtual ~LineOfSightFunction()
            {
            }

            virtual LineOfSightFunction* clone() const = 0;

            virtual std::pair<std::shared_ptr<Action>, Cost> operator() (const State& stateA, const State& stateB) const = 0;
    };

}
}

#endif // LINEOFSIGHTFUNCTION_HPP
