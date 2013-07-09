// Linus van Elswijk 0710261

#ifndef SEARCH_ALGORITHMS_NODE_HPP
#define SEARCH_ALGORITHMS_NODE_HPP

#include <set>
#include <memory>
#include <stdexcept>

extern "C++"
{
namespace searchAlgorithms
{
//////////////////////////////////////////////////////////////////////////////
// class Node definition													//
//////////////////////////////////////////////////////////////////////////////

	/**
	 * Represents a node in the searchspace of a search algorithm.
	 *
	 * Nodes are used by search algorithms.
	 * Nodes are a of wrapper class around the states of a problem.
	 * Nodes allow the search algorithm to build a graph or tree of the states, with
	 * actions as edges to get from one node to another.
	 *
	 * tparam ActionType The type of the actions to get from one node to the next
	 * tparam CostType the type the cost function returns.
	 *		  Should be a numerical type that represents (a subset of) real numbers.
	 */
    template<typename ProblemType>
	class Node
    {
        public:
            typedef typename ProblemType::State  State;
            typedef typename ProblemType::Action Action;
            typedef typename ProblemType::Cost   Cost;

            typedef std::shared_ptr<Node> Ptr;
            typedef std::shared_ptr<const Node> ConstPtr;

		public:

			/**
			 * Constructor for a root node.
			 * @param[in] state the state that corresponds to the node
			 */
            Node( const State& state );

			/**
			 * Constructor for a node with a parent
			 * @param[in] state the state that corresponds to the node
			 * @param[in] the parent node
			 * @param[in] actionPerformedOnParent the action performed on the parent node's state to reach this node's state
			 * @param[in] actionCost the cost of the action performed on the parent node's state to reach this node's state
			 */
            Node( const State &state,
                  const Ptr& parent,
                  const Action& actionPerformedOnParent,
                  const Cost& actionCost
                );

			/**
			 * Copy-constructor of a node
			 * @param[in] otherNode the node that will be copied
			 */
            Node( const Node& otherNode);

			/**
			 * Destructor
			 */
			virtual ~Node();

			/**
			 * Assignment operator,
			 * makes the node copy the value of the argument node.
			 * @return reference to this node
             *
			NodeType& operator= (const NodeType& otherNode);
             */
			/**
			 * Returns the state that is represented by this Node.
			 * @return the state that is represented by this Node
			 */
            const State& getState() const;

			/**
			 * Tells whether the node has a parent or not.
			 * Equivalent to getParent() != NULL.
			 * @return true if the node has a parent,
			 *		   otherwise false.
			 */
            bool hasParent() const;

			/**
			 * Returns a pointer to the parent node.
			 * @return pointer to the parent node or
			 *		   a NULL-pointer if the node doensn't have a parent.
			 */
            Ptr getParent() const;

			/**
			 * Returns the action performed in the parent node's state
			 * to reach this node's state.
			 * Should only be called if the node has a parent.
			 * @return the action performed on the parent node's state
			 */
            const Action& getActionPerformedOnParent() const;

			/**
			 * Returns the cost of the action performed in the parent node's state
			 * to reach this node's state.
			 * Should only be called if the node has a parent.
			 * @return the cost of the action performed on the parent node's state
			 */
            const Cost &getActionCost() const;

			/**
			 * Returns the cost of the path to the node.
			 * @return the cost of the path to the node.
			 */
            const Cost& getPathCost() const;

			/**
			 * Returns the depth of the node in the search tree
			 * @return the depth of the node in the search tree
			 */
            const unsigned int& getDepth() const;

            const bool& hasChildren() const;

            void registerChild(Node* const node);
            void unregisterChild(Node* const node);


            void setParent(const Ptr &parent, const Action &action, const Cost &cost);


		private:
			/**
			 * Returns a dynamicly allocated copy of the actionPerformedOnParent attribute
			 * or NULL if actionPerformedOnParent is a NULL pointer.
			 * @return dynamicly allocated copy of actionPerformedOnParent or NULL
			 */
            Action* actionPerformedOnParentCopy() const;

			/**
			 * Returns a dynamicly allocated copy of the actionCost attribute
			 * or NULL if actionCost is a NULL pointer.
			 * @return dynamicly allocated copy of actionCostCopy or NULL
			 */
            Cost* actionCostCopy() const;

            const State& state;

            unsigned int depth;
            Cost pathCost;

            Ptr parent;
            Action* actionPerformedOnParent;
            Cost *  actionCost;

            std::set<Node*> children;
	};

//////////////////////////////////////////////////////////////////////////////
// member function implementation											//
//////////////////////////////////////////////////////////////////////////////
// public:

    template <typename ProblemType>
    Node<ProblemType>::Node(const State &state )
    :state(state),
	 depth(0),
	 pathCost(0),
     parent(nullptr),
     actionPerformedOnParent(nullptr),
     actionCost(nullptr),
	 children()
	{}

    template <typename ProblemType>
    Node<ProblemType>::Node( const State& state,
                             const Ptr &parent,
                             const Action &actionPerformedOnParent,
                             const Cost &actionCost
                           )
    :state(state),
	 depth(parent->getDepth() + 1),
	 pathCost(parent->getPathCost() + actionCost),
	 parent(parent),
     actionPerformedOnParent(new Action(actionPerformedOnParent)),
     actionCost(new Cost(actionCost)),
	 children()
	{
        this->parent->registerChild(this);
	}

    template <typename ProblemType>
    Node<ProblemType>::Node( const Node<ProblemType>& otherNode
                           )
	:state(otherNode.state),
	 depth(otherNode.depth),
	 pathCost(otherNode.pathCost),
	 parent(otherNode.parent),
     actionPerformedOnParent(nullptr),
     actionCost(nullptr),
	 children()
	{
        if(parent)
		{
            actionPerformedOnParent = new Action(*otherNode.actionPerformedOnParent);
            actionCost = new Cost(otherNode.getActionCost());

            parent->registerChild(this);
		}
	}

    template <typename ProblemType>
    Node<ProblemType>::~Node()
	{
        if(parent)
		{
			delete actionPerformedOnParent;
			delete actionCost;

            parent->unregisterChild(this);
		}
    }

    template <typename ProblemType>
    const typename Node<ProblemType>::State& Node<ProblemType>::getState() const
    {	return state;
    }

    template <typename ProblemType>
    bool Node<ProblemType>::hasParent() const
    {	return parent != nullptr;
	}

    template <typename ProblemType>
    typename Node<ProblemType>::Ptr Node<ProblemType>::getParent() const
	{	return parent;
	}

    template <typename ProblemType>
    const typename Node<ProblemType>::Action& Node<ProblemType>::getActionPerformedOnParent() const
	{
		return *actionPerformedOnParent;
	}

    template <typename ProblemType>
    const typename Node<ProblemType>::Cost& Node<ProblemType>::getActionCost() const
	{	return *actionCost;
	}

    template <typename ProblemType>
    const typename Node<ProblemType>::Cost &Node<ProblemType>::getPathCost() const
	{	
		return pathCost;
	}

    template <typename ProblemType>
    const unsigned int& Node<ProblemType>::getDepth() const
	{	
       return depth;
	}

    template <typename ProblemType>
    const bool& Node<ProblemType>::hasChildren() const
	{
		return children.size() > 0;
	}


    template <typename ProblemType>
    void Node<ProblemType>::registerChild(Node* const node)
	{
        children.insert(node);
	}

    template <typename ProblemType>
    void Node<ProblemType>::unregisterChild(Node* const node)
	{
        children.erase(node);
	}

    template <typename ProblemType>
    void Node<ProblemType>::setParent(const Ptr& parent, const Action& action, const Cost& cost) {
        if(this->hasChildren())
            throw std::runtime_error("Can't set parent of node with children.");

        if(this->hasParent()) {
            this->parent->unregisterChild(this);
            delete this->actionPerformedOnParent;
            delete this->actionCost;
        }

        this->parent = parent;
        this->depth = parent->getDepth() + 1;
        this->actionPerformedOnParent = new Action(action);
        this->actionCost = new Cost(cost);
        parent->registerChild(this);
    }

//////////////////////////////////////////////////////////////////////////////
// private:

    template <typename ProblemType>
    typename Node<ProblemType>::Action* Node<ProblemType>::actionPerformedOnParentCopy() const
	{
        if(actionPerformedOnParent)
            return new Action(*actionPerformedOnParent);
		else
            return nullptr;
	}

    template <typename ProblemType>
    typename Node<ProblemType>::Cost* Node<ProblemType>::actionCostCopy() const
	{
        if(actionCost)
            return new Cost(*actionCost);
		else
            return nullptr;
	}



    /*
    template <typename StateType, typename ActionType, typename CostType>
    void Node<StateType, ActionType, CostType>::setParent(const NodePtr& parent, const ActionType& action, const CostType& cost)
    {
        if(hasChildren())
        {
            throw new std::runtime_error("Can't set parent of node with children");
        }

        if(this->parent)
        {
            this->parent->unregisterChild(this);
        }

        this->parent = parent;
        parent->registerChild(this);

        if(!actionCost)
        {
            actionCost = new CostType(cost);
            actionPerformedOnParent = new ActionType(action);
            depth = parent->depth + 1;
            pathCost = parent->getPathCost() + *actionCost;;
        }
        else
        {
            *actionCost = cost;
            *actionPerformedOnParent = action;
            depth = parent->depth + 1;
        }

        pathCost = parent->getPathCost() + *actionCost;
    }
    */


    /*
    template <typename StateType, typename ActionType, typename CostType>
    Node<StateType, ActionType, CostType>& Node<StateType, ActionType, CostType>::operator= (const Node<StateType, ActionType, CostType>& otherNode)
    {
        if(hasChildren())
        {
                    throw new std::runtime_error("Can't set parent of node with children");
        }

        if(this != &otherNode) {
            state = otherNode.state;
            pathCost = otherNode.pathCost;
            depth = otherNode.depth;

            if(parent)
            {
                delete actionPerformedOnParent;
                actionPerformedOnParent = nullptr;
                delete actionCost;
                actionCost = nullptr;

                parent->unregisterChild(this);
            }

            parent = otherNode.parent;

            if(parent)
            {
                actionPerformedOnParent = new ActionType(*otherNode.actionPerformedOnParent);
                actionCost = new CostType(*otherNode.actionCost);

                parent->registerChild(this);
            }

            return *this;
        }
    }
    */
}
}
#endif
