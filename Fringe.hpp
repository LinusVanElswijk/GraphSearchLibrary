#ifndef FRINGE_HPP
#define FRINGE_HPP

#include <list>
#include <unordered_map>

#include "Node.hpp"
#include "NodeListeners.hpp"

extern "C++"
{
namespace searchAlgorithms
{	
    template<typename ProblemType, typename Hasher>
	class Fringe
    {
        public:
            typedef Node<ProblemType> NodeType;
            typedef typename NodeType::Ptr NodePtr;

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
             typedef NodePushListener<ProblemType>   NodePushListenerType;
             typedef NodePopListener<ProblemType> 	NodePopListenerType;
             typedef NodeUpdateListener<ProblemType> NodeUpdateListenerType;

             typedef std::unordered_map<const State*, NodePtr, Hasher, ClosedListEqualTest> MapOfNodes;

        public:
            typedef typename MapOfNodes::size_type size_type;

            virtual ~Fringe();

            virtual void pushToQueue(const NodePtr& node) = 0;
            virtual NodePtr popFromQueue() = 0;
            virtual void clearQueue() = 0;

            virtual bool replace(const NodePtr& oldNode, const NodePtr& newNode) = 0;

            void push(const NodePtr& node);
            NodePtr pop();
            void clear();

            //virtual void update(const NodePtr& oldNodeValue, const NodePtr& newNodeValue) = 0;

            void push(const std::list<NodePtr> &newNodes);

            size_type size() const;
			bool empty() const;
			
            virtual NodePtr find(const State& state);

			virtual void addNodePushListener(NodePushListenerType& listener);
			virtual void removeNodePushListener(NodePushListenerType& listener);

			virtual void addNodePopListener(NodePopListenerType& listener);
			virtual void removeNodePopListener(NodePopListenerType& listener);

			virtual void addNodeUpdateListener(NodeUpdateListenerType& listener);
			virtual void removeNodeUpdateListener(NodeUpdateListenerType& listener);

		protected:
            void notifyNodePushListeners(const NodePtr& node);
            void notifyNodePopListeners(const NodePtr& node);
            void notifyNodeUpdateListeners(const NodePtr& oldNodeValue, const NodePtr& newNodeValue);

            MapOfNodes nodesInFringe;
        private:

			std::list<NodePushListenerType*> nodePushListenerList;
			std::list<NodePopListenerType*> nodePopListenerList;
			std::list<NodeUpdateListenerType*> nodeUpdateListenerList;

	};
	
    template<typename ProblemType, typename Hasher>
    Fringe<ProblemType,Hasher>::~Fringe()
	{
    }

    template<typename ProblemType, typename Hasher>
    void Fringe<ProblemType,Hasher>::push(const NodePtr& newNode)
    {
      NodePtr oldNode = this->find(newNode->getState());

      if(oldNode == nullptr || this->replace(oldNode, newNode)) {
        if(oldNode != nullptr) {
            this->notifyNodeUpdateListeners(oldNode, newNode);
            this->nodesInFringe.erase(&oldNode->getState());
        }

        this->pushToQueue(newNode);
        nodesInFringe.emplace(&newNode->getState(), newNode);
        this->notifyNodePushListeners(newNode);
      }
    }

    template<typename ProblemType, typename Hasher>
    typename Fringe<ProblemType,Hasher>::NodePtr Fringe<ProblemType,Hasher>::pop()
    {
        NodePtr node = this->popFromQueue();
        nodesInFringe.erase(&node->getState());

        this->notifyNodePopListeners(node);

        return node;
    }

    template<typename ProblemType, typename Hasher>
    void Fringe<ProblemType,Hasher>::clear()
    {
        this->clearQueue();
        this->nodesInFringe.clear();
    }



    template<typename ProblemType, typename Hasher>
    void Fringe<ProblemType,Hasher>::push(const std::list<NodePtr> &newNodes)
    {
        for(auto node: newNodes)
        {
            this->push(node);
        }
    }

    template<typename ProblemType, typename Hasher>
    typename Fringe<ProblemType,Hasher>::size_type Fringe<ProblemType,Hasher>::size() const {
        return this->nodesInFringe.size();
    }

    template<typename ProblemType, typename Hasher>
    bool Fringe<ProblemType,Hasher>::empty() const
	{
        return this->size() == size_type(0);
	}

    template<typename ProblemType, typename Hasher>
    typename Fringe<ProblemType,Hasher>::NodePtr Fringe<ProblemType,Hasher>::find(const State& state)
    {
        auto iterator = nodesInFringe.find(&state);
        if(iterator != nodesInFringe.end())
            return iterator->second;
        else
            return nullptr;
    }

    template<typename ProblemType, typename Hasher>
    void Fringe<ProblemType,Hasher>::notifyNodePushListeners(const NodePtr& node)
	{
        for (auto listener : nodePushListenerList)
		{
            listener->onNodePushed(node);
		}
	}

    template<typename ProblemType, typename Hasher>
    void Fringe<ProblemType,Hasher>::notifyNodePopListeners(const NodePtr& node)
	{
        for (auto listener : nodePopListenerList)
        {
            listener->onNodePopped(node);
        }
	}

    template<typename ProblemType, typename Hasher>
    void Fringe<ProblemType,Hasher>::notifyNodeUpdateListeners(const NodePtr& oldNodeValue, const NodePtr& newNodeValue)
	{
        for (auto listener : nodeUpdateListenerList)
        {
            listener->onNodeUpdated(oldNodeValue, newNodeValue);
        }
	}

    template<typename ProblemType, typename Hasher>
    void Fringe<ProblemType,Hasher>::addNodePushListener(NodePushListenerType& listener)
	{
        bool foundInList = false;

        for (auto iterator = nodePushListenerList.begin(); iterator != nodePushListenerList.end() && !foundInList; iterator++)
		{
            foundInList = (*iterator == &listener);
		}

		if(!foundInList)
		{
			nodePushListenerList.push_back(&listener);
		}
	}

    template<typename ProblemType, typename Hasher>
    void Fringe<ProblemType, Hasher>::removeNodePushListener(NodePushListenerType& listener)
	{
		nodePushListenerList.remove(&listener);
	}

    template<typename ProblemType, typename Hasher>
    void Fringe<ProblemType,Hasher>::addNodePopListener(NodePopListenerType& listener)
	{
        bool foundInList = false;
        for (auto iterator = nodePopListenerList.begin(); iterator != nodePopListenerList.end() && !foundInList; iterator++)
		{
			foundInList = (*iterator == &listener);
		}

		if(!foundInList)
		{
			nodePopListenerList.push_back(&listener);
		}
	}

    template<typename ProblemType, typename Hasher>
    void Fringe<ProblemType,Hasher>::removeNodePopListener(NodePopListenerType& listener)
	{
		nodePopListenerList.remove(&listener);
	}

    template<typename ProblemType, typename Hasher>
    void Fringe<ProblemType,Hasher>::addNodeUpdateListener(NodeUpdateListenerType& listener)
	{
        bool foundInList = false;
        for (auto iterator = nodeUpdateListenerList.begin(); iterator != nodeUpdateListenerList.end() && !foundInList; iterator++)
		{
			foundInList = (*iterator == &listener);
		}

		if(!foundInList)
		{
			nodeUpdateListenerList.push_back(&listener);
		}
	}

    template<typename ProblemType, typename Hasher>
    void Fringe<ProblemType,Hasher>::removeNodeUpdateListener(NodeUpdateListenerType& listener)
	{
		nodeUpdateListenerList.remove(&listener);
	}
}
}
#endif
