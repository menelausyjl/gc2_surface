#ifndef GCS_SCENE_H
#define GCS_SCENE_H

// #include "types.h"
#include "graph.h"
//#include "graph_cut/IBFS.h"
#include "max_flow.h"

namespace yjl
{
//    template <typename NType, typename VType>
//    class MaxFlow
//    {
//    public:
//        // Type-Definitions
//        typedef NType node_type;
//        typedef VType value_type;
//        typedef IBFS::IBFSGraph graph_type;
//
//    public:
//        MaxFlow(size_t numNodes) {
//            graph.initSize((int)numNodes, (int)numNodes*2);
//        }
//
//        inline void AddNode(node_type n, value_type source, value_type sink) {
//            assert(std::isfinite(source) && source >= 0 && std::isfinite(sink) && sink >= 0);
//            graph.addNode((int)n, source, sink);
//        }
//
//        inline void AddEdge(node_type n1, node_type n2, value_type capacity, value_type reverseCapacity) {
//            assert(std::isfinite(capacity) && capacity >= 0 && std::isfinite(reverseCapacity) && reverseCapacity >= 0);
//            graph.addEdge((int)n1, (int)n2, capacity, reverseCapacity);
//        }
//
//        value_type ComputeMaxFlow() {
//            graph.initGraph();
//            return graph.computeMaxFlow();
//        }
//
//        inline bool IsNodeOnSrcSide(node_type n) const {
//            return graph.isNodeOnSrcSide((int)n);
//        }
//
//    protected:
//        graph_type graph;
//    };

    class Scene {
    public:
        struct Options {
            static constexpr CapacityT k_inf = (CapacityT)1e6f;
            CapacityT thickness_sigma = 0.95f;
            CapacityT alpha_vis = 1.f;
        };

        Options options;
        std::shared_ptr<Triangulation> m_graph;
        std::vector<Camera> m_cameras;

        Scene();

        FT calculateMedianThickness() const;

        void build(CapacityT thickness_sigma);

        void buildForward();

        void computeMaxFlow();
    };
} // namespace yjl


#endif