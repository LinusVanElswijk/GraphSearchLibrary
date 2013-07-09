#ifndef FIFO_FRINGE_HPP
#define FIFO_FRINGE_HPP

#include "Fringe.hpp"
#include<queue>

extern "C++"
{
namespace searchAlgorithms
{
    template<typename ProblemType, typename Hasher>
    class FifoFringe: public Fringe<ProblemType,Hasher>
    {
        public:
            typedef typename Node<ProblemType>::Ptr NodePtr;

        public:
            FifoFringe();
            virtual ~FifoFringe();

            virtual void pushToQueue(const NodePtr& node);
            virtual NodePtr popFromQueue();
            virtual void clearQueue();

        private:
            std::queue<NodePtr> queue;
    };

    template<typename ProblemType, typename Hasher>
    FifoFringe<ProblemType, Hasher>::FifoFringe()
    :queue()
    {

    }

    template<typename ProblemType, typename Hasher>
    FifoFringe<ProblemType, Hasher>::~FifoFringe()
    {

    }

    template<typename ProblemType, typename Hasher>
    void FifoFringe<ProblemType, Hasher>::pushToQueue(const NodePtr& node)
    {
        queue.push(node);
    }

    template<typename ProblemType, typename Hasher>
    typename FifoFringe<ProblemType, Hasher>::NodePtr FifoFringe<ProblemType, Hasher>::popFromQueue()
    {
        NodePtr node(queue.front());
        queue.pop();
        return node;
    }

    template<typename ProblemType, typename Hasher>
    void FifoFringe<ProblemType, Hasher>::clearQueue()
    {
        while(queue.size() > 0)
            queue.pop();
    }
}
}

#endif // FIFO_FRINGE_HPP
