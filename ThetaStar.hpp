#ifndef THETA_STAR_HPP
#define THETA_STAR_HPP

#include <sstream>

#include "SearchAlgorithm.hpp"
#include "HeuristicFunction.hpp"
#include "LineOfSightFunction.hpp"
#include "AStarFringe.hpp"

extern "C++"
{
namespace searchAlgorithms
{
    template <typename ProblemType, typename Hasher>
    class ThetaStar: public SearchAlgorithm<ProblemType, Hasher>
    {
        public:
            typedef typename ProblemType::State  State;
            typedef typename ProblemType::Action Action;
            typedef typename ProblemType::Cost   Cost;

            typedef HeuristicFunction<State, Cost> HeuristicFunctionType;
            typedef LineOfSightFunction<ProblemType> LineOfSight;

            typedef typename SearchAlgorithm<ProblemType, Hasher>::NodeType NodeType;
            typedef typename NodeType::Ptr NodePtr;
            typedef typename NodeType::ConstPtr ConstNodePtr;

            typedef typename SearchAlgorithm<ProblemType, Hasher>::SolutionType SolutionType;
            typedef typename SearchAlgorithm<ProblemType, Hasher>::SolutionPtr SolutionPtr;

            ThetaStar(const HeuristicFunctionType& heuristicFunction, const LineOfSight& lineOfSight);

            virtual ~ThetaStar();

           /**
            *  The A* search algorithm based on graph search.
            *
            *  Like graph search, this variant of A* keeps a list of the states that have been visited before.
            *  If A* encounters a state for a second time, it will keep only the node with the shortest path so far in the list
            *  and ignore the other Node.
            *
            *
            *  @param[in] problem the problem for which a solution will be searched
            *  @param[in] heuristicFunction the heuristic function that is used to estimate how far a state is from a goal state.
            *  @tparam StateType the type of the states of the problem
            *  @tparam ActionType the type of the actions that can be made to get from one state to the next
            *  @tparem CostType the type of the costs of the actions
            *  @return The solution, which is a path from the problem node to the goal node or
            *			NULL if no solution was found.
            */
            SolutionPtr operator() ( const ProblemType& problem );

        protected:
            typedef typename SearchAlgorithm<ProblemType, Hasher>::FringeType FringeType;

            virtual FringeType* createFringe();

            //virtual bool replaceIfPossible(const NodePtr& newNode);

            //virtual	bool replaceIfPossible(const std::list< const NodePtr& >& newNodes);

            //Cost evaluation( const ConstNodePtr& node );


        private:
            HeuristicFunctionType* heuristicFunction;
            LineOfSight* lineOfSight;
    };

    template <typename ProblemType, typename Hasher>
    ThetaStar<ProblemType, Hasher>::ThetaStar(const HeuristicFunction<State, Cost>& heuristicFunction, const LineOfSight& lineOfSight)
    :SearchAlgorithm<ProblemType, Hasher>(),
     heuristicFunction( heuristicFunction.clone() ),
     lineOfSight(lineOfSight.clone())
    {
        this->fringe = this->createFringe();
    }

    template <typename ProblemType, typename Hasher>
    ThetaStar<ProblemType, Hasher>::~ThetaStar()
    {
        delete heuristicFunction;
        delete lineOfSight;
    }

    template <typename ProblemType, typename Hasher>
    typename ThetaStar<ProblemType, Hasher>::SolutionPtr ThetaStar<ProblemType, Hasher>::operator() ( const ProblemType& problem )
    {
        SolutionPtr solution(nullptr);
        this->visits = 0;
        this->maxNodesInMemory = 0;

        NodePtr initialStateNode = this->createNode( problem.initialState() );

        this->fringe->push(initialStateNode);

        //clock_t begin = clock();

        while( !solution && !this->fringe->empty() )
        {
            if(this->fringe->size() + this->closedList.size() > this->maxNodesInMemory)
                this->maxNodesInMemory = this->fringe->size() + this->closedList.size();
            this->visits++;

            NodePtr currentNode = this->fringe->pop();
            this->notifyNodeVisitListeners(currentNode);

            if( problem.isGoal( currentNode->getState() ) )
            {
                solution = SolutionPtr(new SolutionType(currentNode));
                continue;
            }

            std::pair<const State*, NodePtr> entry(&currentNode->getState(), currentNode);
            this->closedList.insert(entry);
            this->notifyNodeClosureListeners(currentNode);

            std::list<NodePtr>&& successors = this->expand(currentNode, problem);

            for(NodePtr node: successors) {
                auto nodeIt = this->closedList.find(&node->getState());

                if(nodeIt == this->closedList.end()) {

                    if(node->hasParent()) {

                        if(node->getParent()->hasParent()) {
                            auto grandParent = node->getParent()->getParent();

                            auto actionAndCost = (*lineOfSight)(node->getState(), grandParent->getState());
                            if(actionAndCost.first != nullptr) {
                                node->setParent(grandParent, *actionAndCost.first, actionAndCost.second);
                            }
                        }
                    }

                    this->fringe->push(node);
                }
            }
        }

        //clock_t end = clock();

        //double timeSpend = (double)(end - begin) / CLOCKS_PER_SEC;
        //std::cout << "Spent " << timeSpend << " seconds on " << visit << " visits (" << visit/timeSpend << " n/s)" << std::endl;

        this->cleanUp();
        return solution;
    }

    template <typename ProblemType, typename Hasher>
    typename ThetaStar<ProblemType, Hasher>::FringeType* ThetaStar<ProblemType, Hasher>::createFringe()
    {
        return new AStarFringe<ProblemType, Hasher>(*heuristicFunction);
    }
    /*
    template <typename ProblemType, typename Hasher>
    bool AStar<ProblemType, Hasher>::replaceIfPossible( const NodePtr& newNode )
    {
        NodePtr fringeNode = this->fringe.find(newNode->getState());

        if(fringeNode != NULL )
        {
            if( newNode->getPathCost() < fringeNode->getPathCost() )
            {
                this->fringe->update(fringeNode, newNode);
                return true;
            }

            return true;
        }

        typedef typename searchAlgorithms::AStar<ProblemType, Hasher>::ClosedListType::iterator Iterator;
        Iterator iterator = this->closedList.find(newNode);

        if( iterator != this->closedList.end() )
        {
            NodeType* closedListNode = *iterator;

            if( newNode -> getPathCost() < closedListNode->getPathCost() )
            {
                this->closedList.erase(closedListNode);
                *closedListNode = *newNode;
                this->fringe->push(closedListNode);
            }

            return true;
        }

        return false;
    }

    template <typename ProblemType, typename Hasher>
    bool AStar<ProblemType, Hasher>::replaceIfPossible(const std::list< const NodePtr& >& newNodes )
    {
        typedef typename std::list< NodeType* >::iterator Iterator;

        bool replacedSomething = false;

        Iterator newNodeIterator = newNodes.begin();
        while( newNodeIterator != newNodes.end() )
        {
            bool replaced = replaceIfPossible(*newNodeIterator);

            if(replaced)
            {
                replacedSomething = true;
                deleteNode(*newNodeIterator);
            }
            else
            {
                this->fringe->push(*newNodeIterator);
            }

            newNodeIterator++;
        }

        return replacedSomething;
    }
    *
    template <typename ProblemType, typename Hasher>
    typename AStar<ProblemType, Hasher>::Cost AStar<ProblemType, Hasher>::evaluation( const ConstNodePtr& node )
    {
        return node->getPathCost() + heuristicFunction(node->getState());
    }
    */
}
}

#endif
