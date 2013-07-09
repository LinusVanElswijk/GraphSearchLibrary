#ifndef SEARCH_ALGORITHM_HPP
#define SEARCH_ALGORITHM_HPP

#include <list>
#include <unordered_map>
//#include <map>
//#include <unordered_set>
#include <memory>


#include "Node.hpp"
#include "NodeListeners.hpp"
#include "Problem.hpp"
#include "Solution.hpp"
#include "Fringe.hpp"

extern "C++"
{
namespace searchAlgorithms
{
	/**
	 * Abstract base class for all search algorithms.
	 * This base class handles all function hooks.
	 */
    template<typename ProblemType, typename Hasher>
	class SearchAlgorithm
    {
        public:
            typedef typename ProblemType::State  State;
            typedef typename ProblemType::Action Action;
            typedef typename ProblemType::Cost   Cost;

        private:
            class ClosedListEqualTest {
                public:

                bool operator () (const State* const A, const State* const B) const {
                    return *A == *B;
                }
            };

		protected:


            typedef Node<ProblemType>           NodeType;
            typedef typename NodeType::Ptr      NodePtr;
            typedef typename NodeType::ConstPtr ConstNodePtr;

            typedef Solution<ProblemType> SolutionType;
            typedef std::shared_ptr<Solution<ProblemType>> SolutionPtr;

            typedef Fringe<ProblemType,Hasher>                 FringeType;
            typedef std::unordered_map<const State*, NodePtr, Hasher, ClosedListEqualTest> ClosedListType;
			
            typedef NodeVisitListener<ProblemType>          NodeVisitListenerType;
            typedef NodeExpansionListener<ProblemType> 		NodeExpansionListenerType;
            typedef NodeClosureListener<ProblemType>   		NodeClosureListenerType;
            typedef NodePushListener<ProblemType>      		NodePushListenerType;
            typedef NodePopListener<ProblemType> 	   		NodePopListenerType;
            typedef NodeUpdateListener<ProblemType>    		NodeUpdateListenerType;
		
		public:
			SearchAlgorithm();
			virtual ~SearchAlgorithm();
			
			virtual void addNodeVisitListener(NodeVisitListenerType& listener);
			virtual void removeNodeVisitListener(NodeVisitListenerType& listener);
			
			virtual void addNodeExpansionListener(NodeExpansionListenerType& listener);
			virtual void removeNodeExpansionListener(NodeExpansionListenerType& listener);
			
			virtual void addNodeClosureListener(NodeClosureListenerType& listener);
			virtual void removeNodeClosureListener(NodeClosureListenerType& listener);
			
            virtual void addNodePushListener(NodePushListenerType& listener);
			virtual void removeNodePushListener(NodePushListenerType& listener);

			virtual void addNodePopListener(NodePopListenerType& listener);
			virtual void removeNodePopListener(NodePopListenerType& listener);

			virtual void addNodeUpdateListener(NodeUpdateListenerType& listener);
			virtual void removeNodeUpdateListener(NodeUpdateListenerType& listener);

			unsigned int getFringeSize() const;
			bool fringeIsEmpty() const;

			unsigned int getClosedListSize() const;

            virtual SolutionPtr operator() (const ProblemType& problem) = 0;

            unsigned int getNrOfVisits() {
                return visits;
            }

            unsigned int getMaxNodesInMemory() {
                return maxNodesInMemory;
            }

		protected:
            virtual std::list< NodePtr > expand(NodePtr &node, const ProblemType& problem);
			
            void notifyNodeVisitListeners(const ConstNodePtr& node);
            void notifyNodeExpansionListeners(const ConstNodePtr& node);
            void notifyNodeClosureListeners(const ConstNodePtr& node);
			
            virtual NodePtr createNode( const State& state );
            virtual NodePtr createNode( const State& state, const NodePtr& parent, const Action& actionPerformedOnParent, const Cost& actionCost);
            virtual NodePtr createNode( const NodePtr& otherNode);

			void cleanUp();
			
            void addToClosedList(const NodePtr& node);
            void removeFromClosedList(const NodePtr& node);
            bool isInClosedList(const NodePtr& node);

			virtual FringeType* createFringe() = 0;

            FringeType* fringe;
            ClosedListType closedList;

            unsigned int visits;
            unsigned int maxNodesInMemory;

        private:
			std::list<NodeVisitListenerType*> nodeVisitListenerList;
			std::list<NodeExpansionListenerType*> nodeExpansionListenerList;
            std::list<NodeClosureListenerType*> nodeClosureListenerList;
	};
	
    template<typename ProblemType, typename Hasher>
    SearchAlgorithm<ProblemType, Hasher>::SearchAlgorithm()
    :fringe(),
	 closedList(),
	 nodeVisitListenerList(),
	 nodeExpansionListenerList(),
     nodeClosureListenerList()
    {
	}
	
    template<typename ProblemType, typename Hasher>
    SearchAlgorithm<ProblemType, Hasher>::~SearchAlgorithm()
	{
        this->cleanUp();
        delete fringe;
	}
	
    template<typename ProblemType, typename Hasher>
    typename std::list< typename Node<ProblemType>::Ptr > SearchAlgorithm<ProblemType, Hasher>::expand(NodePtr& node, const ProblemType& problem)
	{
        notifyNodeExpansionListeners(node);
		
        typedef typename ProblemType::ActionAndResult ActionAndResult;
        typedef typename std::list<NodePtr>::size_type ElementNr;
		typedef typename std::list<ActionAndResult>::iterator Iterator;

        std::list<NodePtr> successorNodes;
        std::list<ActionAndResult>&& actionsAndResults = problem.getSuccessors(node->getState());

        for(Iterator iterator = actionsAndResults.begin(); iterator != actionsAndResults.end(); ++iterator)
        {
            NodePtr&& successor = createNode( iterator->result,
                                              node,
                                              iterator->action,
                                              problem.actionCost(node->getState(), iterator->action)
                                            );

            successorNodes.push_back(successor);
		}

		return successorNodes;
	}

    template<typename ProblemType, typename Hasher>
    unsigned int SearchAlgorithm<ProblemType, Hasher>::getFringeSize() const
	{
		return fringe->size();
	}

    template<typename ProblemType, typename Hasher>
    bool SearchAlgorithm<ProblemType, Hasher>::fringeIsEmpty() const
	{
		return fringe->isEmpty();
	}

    template<typename ProblemType, typename Hasher>
    unsigned int SearchAlgorithm<ProblemType, Hasher>::getClosedListSize() const
	{
		return closedList.size();
	}

    template<typename ProblemType, typename Hasher>
    void SearchAlgorithm<ProblemType, Hasher>::notifyNodeVisitListeners(const ConstNodePtr& node)
	{
		typedef typename std::list<NodeVisitListenerType*>::iterator Iterator;
		
		for (Iterator iterator = nodeVisitListenerList.begin(); iterator != nodeVisitListenerList.end(); iterator++)
		{
			(*iterator)->onNodeVisited(node);
		}
	}
	
    template<typename ProblemType, typename Hasher>
    void SearchAlgorithm<ProblemType, Hasher>::notifyNodeExpansionListeners(const ConstNodePtr& node)
	{
		typedef typename std::list<NodeExpansionListenerType*>::iterator Iterator;
		
		for (Iterator iterator = nodeExpansionListenerList.begin(); iterator != nodeExpansionListenerList.end(); iterator++)
		{
			(*iterator)->onNodeExpanded(node);
		}
	}
	
    template<typename ProblemType, typename Hasher>
    void SearchAlgorithm<ProblemType, Hasher>::notifyNodeClosureListeners(const ConstNodePtr& node)
	{
		typedef typename std::list<NodeClosureListenerType*>::iterator Iterator;
		
		for (Iterator iterator = nodeClosureListenerList.begin(); iterator != nodeClosureListenerList.end(); iterator++)
		{
			(*iterator)->onNodeClosed(node);
		}
	}
	
    template<typename ProblemType, typename Hasher>
    void SearchAlgorithm<ProblemType, Hasher>::addNodeVisitListener(NodeVisitListenerType& listener)
	{	
		typedef typename std::list<NodeVisitListenerType*>::iterator Iterator;
		
		bool foundInList = false;
		for (Iterator iterator = nodeVisitListenerList.begin(); iterator != nodeVisitListenerList.end() && !foundInList; iterator++)
		{
			foundInList = (*iterator == &listener);
		}
		
		if(!foundInList)
		{
			nodeVisitListenerList.push_back(&listener);
		}
	}
	
    template<typename ProblemType, typename Hasher>
    void SearchAlgorithm<ProblemType, Hasher>::removeNodeVisitListener(NodeVisitListenerType& listener)
	{	
		nodeVisitListenerList.remove(&listener);
	}
	
    template<typename ProblemType, typename Hasher>
    void SearchAlgorithm<ProblemType, Hasher>::addNodeExpansionListener(NodeExpansionListenerType& listener)
	{	
		typedef typename std::list<NodeExpansionListenerType*>::iterator Iterator;
		
		bool foundInList = false;
		for (Iterator iterator = nodeExpansionListenerList.begin(); iterator != nodeExpansionListenerList.end() && !foundInList; iterator++)
		{
			foundInList = (*iterator == &listener);
		}
		
		if(!foundInList)
		{
			nodeExpansionListenerList.push_back(&listener);
		}
	}
	
    template<typename ProblemType, typename Hasher>
    void SearchAlgorithm<ProblemType, Hasher>::removeNodeExpansionListener(NodeExpansionListenerType& listener)
	{	
		nodeExpansionListenerList.remove(&listener);
    }
	
    template<typename ProblemType, typename Hasher>
    void SearchAlgorithm<ProblemType, Hasher>::addNodeClosureListener(NodeClosureListenerType& listener)
	{	
		typedef typename std::list<NodeClosureListenerType*>::iterator Iterator;
		
		bool foundInList = false;
		for (Iterator iterator = nodeClosureListenerList.begin(); iterator != nodeClosureListenerList.end() && !foundInList; iterator++)
		{
			foundInList = (*iterator == &listener);
		}
		
		if(!foundInList)
		{
			nodeClosureListenerList.push_back(&listener);
		}
	}
	
    template<typename ProblemType, typename Hasher>
    void SearchAlgorithm<ProblemType, Hasher>::removeNodeClosureListener(NodeClosureListenerType& listener)
	{	
		nodeClosureListenerList.remove(&listener);
	}
	
    template<typename ProblemType, typename Hasher>
    void SearchAlgorithm<ProblemType, Hasher>::addNodePushListener(NodePushListenerType& listener)
	{
		this->fringe->addNodePushListener(listener);
	}

    template<typename ProblemType, typename Hasher>
    void SearchAlgorithm<ProblemType, Hasher>::removeNodePushListener(NodePushListenerType& listener)
	{
		this->fringe->removeNodePushListener(listener);
	}

    template<typename ProblemType, typename Hasher>
    void SearchAlgorithm<ProblemType, Hasher>::addNodePopListener(NodePopListenerType& listener)
	{
		this->fringe->addNodePopListener(listener);
	}

    template<typename ProblemType, typename Hasher>
    void SearchAlgorithm<ProblemType, Hasher>::removeNodePopListener(NodePopListenerType& listener)
	{
		this->fringe->removeNodePopListener(listener);
	}

    template<typename ProblemType, typename Hasher>
    void SearchAlgorithm<ProblemType, Hasher>::addNodeUpdateListener(NodeUpdateListenerType& listener)
	{
		this->fringe->addNodeUpdateListener(listener);
	}

    template<typename ProblemType, typename Hasher>
    void SearchAlgorithm<ProblemType, Hasher>::removeNodeUpdateListener(NodeUpdateListenerType& listener)
	{
		this->fringe->removeNodeUpdateListener(listener);
	}

    template<typename ProblemType, typename Hasher>
    typename Node<ProblemType>::Ptr SearchAlgorithm<ProblemType, Hasher>::createNode( const State& state )
	{	
        return NodePtr(new NodeType(state));
	}
	
    template<typename ProblemType, typename Hasher>
    typename Node<ProblemType>::Ptr SearchAlgorithm<ProblemType, Hasher>::createNode(const State& state, const NodePtr &parent, const Action& actionPerformedOnParent, const Cost& actionCost)
	{
        return NodePtr(new NodeType(state, parent, actionPerformedOnParent, actionCost));
	}
	
    template<typename ProblemType, typename Hasher>
    typename Node<ProblemType>::Ptr SearchAlgorithm<ProblemType, Hasher>::createNode( const NodePtr& otherNode)
	{
        return NodePtr(new NodeType(*otherNode));
	}
	
    template<typename ProblemType, typename Hasher>
    void SearchAlgorithm<ProblemType, Hasher>::cleanUp()
	{
        this->fringe->clear();
        closedList.clear();
    }
}
}
#endif
