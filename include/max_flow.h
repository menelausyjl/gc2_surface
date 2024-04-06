//
// Created by Yujie Li on 2024/3/30.
//

#ifndef GC3_MAX_FLOW_H
#define GC3_MAX_FLOW_H

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/one_bit_color_map.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/boykov_kolmogorov_max_flow.hpp>

namespace yjl {
template <typename NType, typename VType>
class MaxFlow
{
public:
    // Type-Definitions
    typedef NType node_type;
    typedef VType value_type;
    typedef boost::vecS out_edge_list_t;
    typedef boost::vecS vertex_list_t;
    typedef boost::adjacency_list_traits<out_edge_list_t, vertex_list_t, boost::directedS> graph_traits;
    typedef typename graph_traits::edge_descriptor edge_descriptor;
    typedef typename graph_traits::vertex_descriptor vertex_descriptor;
    typedef typename graph_traits::vertices_size_type vertex_size_type;
    struct Edge {
        value_type capacity;
        value_type residual;
        edge_descriptor reverse;
    };
    typedef boost::adjacency_list<out_edge_list_t, vertex_list_t, boost::directedS, size_t, Edge> graph_type;
    typedef typename boost::graph_traits<graph_type>::edge_iterator edge_iterator;
    typedef typename boost::graph_traits<graph_type>::out_edge_iterator out_edge_iterator;

public:
    MaxFlow(size_t numNodes) : graph(numNodes+2), S(node_type(numNodes)), T(node_type(numNodes+1)) {}

    void AddNode(node_type n, value_type source, value_type sink) {
        assert(std::isfinite(source) && source >= 0 && std::isfinite(sink) && sink >= 0);
        if (source > 0) {
            edge_descriptor e(boost::add_edge(S, n, graph).first);
            edge_descriptor er(boost::add_edge(n, S, graph).first);
            graph[e].capacity = source;
            graph[e].reverse = er;
            graph[er].reverse = e;
        }
        if (sink > 0) {
            edge_descriptor e(boost::add_edge(n, T, graph).first);
            edge_descriptor er(boost::add_edge(T, n, graph).first);
            graph[e].capacity = sink;
            graph[e].reverse = er;
            graph[er].reverse = e;
        }
    }

    void AddEdge(node_type n1, node_type n2, value_type capacity, value_type reverseCapacity) {
        assert(std::isfinite(capacity) && capacity >= 0 && std::isfinite(reverseCapacity) && reverseCapacity >= 0);
        edge_descriptor e(boost::add_edge(n1, n2, graph).first);
        edge_descriptor er(boost::add_edge(n2, n1, graph).first);
        graph[e].capacity = capacity;
        graph[er].capacity = reverseCapacity;
        graph[e].reverse = er;
        graph[er].reverse = e;
    }

    value_type ComputeMaxFlow() {
        vertex_size_type n_verts(boost::num_vertices(graph));
        color.resize(n_verts);
        std::vector<edge_descriptor> pred(n_verts);
        std::vector<vertex_size_type> dist(n_verts);
        return boost::boykov_kolmogorov_max_flow(graph,
                                                 boost::get(&Edge::capacity, graph),
                                                 boost::get(&Edge::residual, graph),
                                                 boost::get(&Edge::reverse, graph),
                                                 &pred[0],
                                                 &color[0],
                                                 &dist[0],
                                                 boost::get(boost::vertex_index, graph),
                                                 S, T
        );
    }

    inline bool IsNodeOnSrcSide(node_type n) const {
        return (color[n] != boost::white_color);
    }

protected:
    graph_type graph;
    std::vector<boost::default_color_type> color;
    const node_type S;
    const node_type T;
};
}
#endif //GC3_MAX_FLOW_H
