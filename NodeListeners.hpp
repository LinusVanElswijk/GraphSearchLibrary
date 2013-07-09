#ifndef NODE_LISTENER_HPP
#define NODE_LISTENER_HPP

#include "Node.hpp"

extern "C++"
{
namespace searchAlgorithms
{	
    template<typename ProblemType>
	class NodeVisitListener
	{	
        public:
            typedef Node<ProblemType> NodeType;

            virtual void onNodeVisited(const typename NodeType::ConstPtr& node) = 0;
	};
	
    template<typename ProblemType>
	class NodeExpansionListener
	{	
        public:
            typedef Node<ProblemType> NodeType;

            virtual void onNodeExpanded(const typename NodeType::ConstPtr& node) = 0;
	};
	
    template<typename ProblemType>
	class NodeClosureListener
	{	
        public:
            typedef Node<ProblemType> NodeType;

            virtual void onNodeClosed(const typename NodeType::ConstPtr& node) = 0;
	};

    template<typename ProblemType>
	class NodePushListener
	{
        public:
            typedef Node<ProblemType> NodeType;

            virtual void onNodePushed(const typename NodeType::ConstPtr& node) = 0;
	};

    template<typename ProblemType>
	class NodePopListener
	{
        public:
            typedef Node<ProblemType> NodeType;

            virtual void onNodePopped(const typename NodeType::ConstPtr& node) = 0;
	};

    template<typename ProblemType>
	class NodeUpdateListener
	{
        public:
            typedef Node<ProblemType> NodeType;

            virtual void onNodeUpdated(const typename NodeType::ConstPtr& oldNodeValue, const typename NodeType::ConstPtr& newNodeValue) = 0;
	};
}
}
#endif
