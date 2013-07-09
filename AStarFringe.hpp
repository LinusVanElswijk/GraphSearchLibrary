#ifndef A_STAR_FRINGE_HPP
#define A_STAR_FRINGE_HPP

#include <set>

#include "Fringe.hpp"
#include "HeuristicFunction.hpp"

extern "C++"
{
namespace searchAlgorithms
{	
    template<typename ProblemType, typename Hasher>
    class AStarFringe: public Fringe<ProblemType, Hasher>
    {
        public:
            typedef typename Fringe<ProblemType,Hasher>::NodePtr NodePtr;

            typedef typename Fringe<ProblemType,Hasher>::State  State;
            typedef typename Fringe<ProblemType,Hasher>::Action Action;
            typedef typename Fringe<ProblemType,Hasher>::Cost   Cost;

        public:
            AStarFringe(const HeuristicFunction<State, Cost> &heuristicFunction);
			virtual ~AStarFringe();
			
            virtual void pushToQueue(const NodePtr& node);
            virtual NodePtr popFromQueue();
            virtual bool replace(const NodePtr& oldNode, const NodePtr& newNode);

            virtual void clearQueue();

		private:
            class NodeComparator
			{
				public:
                    NodeComparator(const HeuristicFunction<typename ProblemType::State, typename ProblemType::Cost>& heuristicFunction);
                    ~NodeComparator();
					
                    bool operator() (const NodePtr& nodeA, const NodePtr& nodeB);
					
                public:
                    Cost evaluation(const NodePtr& node);
					
                    const HeuristicFunction<State, Cost>& heuristicFunction;
			};
			
            typedef std::multiset<NodePtr, NodeComparator> PriorityQueue;
            typedef typename PriorityQueue::iterator QueueIterator;

			NodeComparator nodeComparator;
            PriorityQueue queue;
	};

    template<typename ProblemType, typename Hasher>
    AStarFringe<ProblemType, Hasher>::AStarFringe(const HeuristicFunction<State, Cost> &heuristicFunction)
    :nodeComparator(heuristicFunction),
     queue(nodeComparator)
	{	
	}

    template<typename ProblemType, typename Hasher>
    AStarFringe<ProblemType, Hasher>::~AStarFringe()
	{
	}


    template<typename ProblemType, typename Hasher>
    void AStarFringe<ProblemType, Hasher>::pushToQueue(const NodePtr &node)
	{	
        queue.insert(node);
    }

    template<typename ProblemType, typename Hasher>
    typename AStarFringe<ProblemType, Hasher>::NodePtr AStarFringe<ProblemType, Hasher>::popFromQueue()
	{	
        auto iterator = queue.begin();

        NodePtr ptr = *iterator;
        queue.erase(iterator);

        return ptr;
    }

    template<typename ProblemType, typename Hasher>
    bool AStarFringe<ProblemType, Hasher>::replace(const NodePtr& oldNode, const NodePtr& newNode)
    {
        return this->nodeComparator(newNode, oldNode);
    }

    template<typename ProblemType, typename Hasher>
    void AStarFringe<ProblemType, Hasher>::clearQueue()
	{
        queue.clear();
	}


    template<typename ProblemType, typename Hasher>
    AStarFringe<ProblemType, Hasher>::NodeComparator::NodeComparator(const HeuristicFunction<typename ProblemType::State, typename ProblemType::Cost>& heuristicFunction)
    :heuristicFunction(heuristicFunction)
	{
	}

    template<typename ProblemType, typename Hasher>
    AStarFringe<ProblemType, Hasher>::NodeComparator::~NodeComparator()
    {
    }
	
    template<typename ProblemType, typename Hasher>
    bool AStarFringe<ProblemType, Hasher>::NodeComparator::operator() (const NodePtr& nodeA, const NodePtr& nodeB)
	{
        return ( evaluation(nodeA) <  evaluation(nodeB)	) ||
			   ( evaluation(nodeA) == evaluation(nodeB) &&
                 nodeA->getActionCost() > nodeB->getActionCost()
               );
	}
	
    template<typename ProblemType, typename Hasher>
    typename AStarFringe<ProblemType, Hasher>::Cost AStarFringe<ProblemType, Hasher>::NodeComparator::evaluation(const NodePtr& node)
	{
        return heuristicFunction( node->getState() ) * 1.000001 + node->getPathCost();
	}


    /*
    template<typename StateType, typename ActionType, typename CostType, typename Hasher>
    void AStarFringe<StateType, ActionType, CostType, Hasher>::update(const NodePtr &nodeToUpdate, const NodePtr &newNodeValue)
    {
        this->notifyNodeUpdateListeners(*nodeToUpdate, *newNodeValue);

        typename MapOfNodes::iterator mapIterator = mapOfNodesInQueue.find( nodeToUpdate->getState() );
        QueueIterator queueIterator = mapIterator->second;

        mapOfNodesInQueue.erase(mapIterator);
        queue.erase(queueIterator);

        *nodeToUpdate = *newNodeValue;

        queueIterator = queue.insert(nodeToUpdate);
        mapOfNodesInQueue.insert(std::pair<StateType, QueueIterator>( (*queueIterator)->getState(), queueIterator) );

        insertionTimes.insert(std::pair<StateType, unsigned long>((*queueIterator)->getState(), insertionCounter));
        insertionCounter++;
    }
    */

/*
    template<typename StateType, typename ActionType, typename CostType>
    class AStarFringe<StateType, ActionType, CostType, void>: public Fringe<StateType, ActionType, CostType>
    {
        public:
            AStarFringe(HeuristicFunction<StateType,  CostType>& heuristicFunction);
            virtual ~AStarFringe();

            virtual void push(const NodePtr& node);
            virtual NodePtr pop();
            virtual void update(const NodePtr& oldNodeValue, const NodePtr& newNodeValue);

            virtual void clear();

            virtual unsigned int size() const;

            virtual NodePtr getNodeWithState(const StateType& state);

        private:

            class NodeComparator;

            typedef std::map<NodeType*, unsigned int, NodeComparator> MapOfInsertionTimes;


            class NodeComparator
            {
                public:
                    NodeComparator(HeuristicFunction<StateType, CostType>& heuristicFunction, MapOfInsertionTimes& insertionTimes);

                    bool operator() (const NodeType* nodeA, const NodeType* nodeB);

                private:
                    CostType evaluation(const NodeType* node);

                    HeuristicFunction<StateType, CostType>* heuristicFunction;
                    MapOfInsertionTimes& insertionTimes;
            };


            typedef std::multiset<NodeType*, NodeComparator> PriorityQueue;
            typedef typename PriorityQueue::iterator QueueIterator;

            MapOfInsertionTimes insertionTimes;
            NodeComparator nodeComparator;
            PriorityQueue queue;

            unsigned long insertionCounter;
    };
*/

/*
    template<typename StateType, typename ActionType, typename CostType>
    AStarFringe<StateType, ActionType, CostType, void>::AStarFringe(HeuristicFunction<StateType,  CostType>& heuristicFunction)
    :insertionTimes(),
     nodeComparator(heuristicFunction, insertionTimes),
     queue(nodeComparator),
     insertionCounter(0)
    {
    }
*/

/*
    template<typename StateType, typename ActionType, typename CostType>
    void AStarFringe<StateType, ActionType, CostType, void>::push(NodeType* node)
    {
        queue.insert(node);

        insertionTimes.insert(std::pair<NodeType*, unsigned long>(node, insertionCounter));
        insertionCounter++;

        this->notifyNodePushListeners(*node);
    }
*/

/*
    template<typename StateType, typename ActionType, typename CostType>
    typename AStarFringe<StateType, ActionType, CostType, void>::NodeType* AStarFringe<StateType, ActionType, CostType, void>::pop()
    {
        QueueIterator iterator = queue.begin();
        NodeType* node = *iterator;

        queue.erase(iterator);

        this->notifyNodePopListeners( *node );

        return node;
    }
*/

/*
    template<typename StateType, typename ActionType, typename CostType>
    void AStarFringe<StateType, ActionType, CostType, void>::update(NodeType* nodeToUpdate, NodeType* newNodeValue)
    {
        this->notifyNodeUpdateListeners(*nodeToUpdate, *newNodeValue);

        queue.erase(nodeToUpdate);

        *nodeToUpdate = *newNodeValue;

        queue.insert(nodeToUpdate);

        insertionTimes.insert(std::pair<NodeType*, unsigned long>(nodeToUpdate, insertionCounter));
        insertionCounter++;
    }
*/

/*
    template<typename StateType, typename ActionType, typename CostType>
    typename AStarFringe<StateType, ActionType, CostType, void>::NodeType* AStarFringe<StateType, ActionType, CostType, void>::getNodeWithState(const StateType& state)
    {
        typename PriorityQueue::iterator iterator = queue.begin();

        while( iterator != queue.end() )
        {
            if ( (*iterator)->getState() == state )
                return *iterator;

            iterator++;
        }

        return NULL;
    }
*/
}
}
#endif
