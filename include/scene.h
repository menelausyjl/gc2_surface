#ifndef GCS_SCENE_H
#define GCS_SCENE_H

// #include "types.h"
#include "graph.h"
#include "graph_cut/IBFS.h"

namespace yjl
{
    template <typename NType, typename VType>
    class MaxFlow
    {
    public:
        // Type-Definitions
        typedef NType node_type;
        typedef VType value_type;
        typedef IBFS::IBFSGraph graph_type;

    public:
        MaxFlow(size_t numNodes) {
            graph.initSize((int)numNodes, (int)numNodes*2);
        }

        inline void AddNode(node_type n, value_type source, value_type sink) {
            assert(std::isfinite(source) && source >= 0 && std::isfinite(sink) && sink >= 0);
            graph.addNode((int)n, source, sink);
        }

        inline void AddEdge(node_type n1, node_type n2, value_type capacity, value_type reverseCapacity) {
            assert(std::isfinite(capacity) && capacity >= 0 && std::isfinite(reverseCapacity) && reverseCapacity >= 0);
            graph.addEdge((int)n1, (int)n2, capacity, reverseCapacity);
        }

        value_type ComputeMaxFlow() {
            graph.initGraph();
            return graph.computeMaxFlow();
        }

        inline bool IsNodeOnSrcSide(node_type n) const {
            return graph.isNodeOnSrcSide((int)n);
        }

    protected:
        graph_type graph;
    };

    class Scene {
    public:
        struct Options {
            static constexpr CapacityT k_inf = (CapacityT)1e7f;
            CapacityT alpha_vis = 1.f;
        };

        Options options;
        std::shared_ptr<Triangulation> m_graph;
        std::vector<Camera> m_cameras;

        Scene() {
            m_graph = std::make_shared<Triangulation>();
        }

        void build() {

            const CapacityT alpha_vis = options.alpha_vis;

            for (int i = 0; i < m_cameras.size(); ++i) {
                std::cout << i << "///\n";

                const auto& cam = m_cameras[i];
                const Point& cam_origin = cam.origin;
                for (Vertex_handle vh : cam.visible_points) {
                    std::cout << vh->point() << " ... \n";
                    const Point& point_pos = vh->point();
                    CapacityT view_weight = vh->info()[i];

                    Line_face_circulator face_circ_curr{vh, m_graph.get(), cam_origin};
                    if (face_circ_curr.is_empty() || m_graph->is_infinite(face_circ_curr)) {
                        continue;
                    }
                    Line_face_circulator face_circ_next = face_circ_curr;
                    ++face_circ_next;

//                    std::cout << (face_circ_curr == Face_handle{}) << "!!\n";
//                    for (int ii = 0; ii < 3; ++ii) {
//                        std::cout << face_circ_curr.handle()->vertex(ii)->point() << std::endl;
//                    }


//                    if (face_circ_curr.handle() == m_graph->infinite_face()) {
//                        face_circ_curr->info().s += Options::k_inf;
//                        continue;
//                    }

//                    Line_face_circulator face_circ_end = m_graph->line_walk(cam_origin, point_pos);

//                    --face_circ_curr;
//                    face_circ_curr->info().t += alpha_vis * view_weight;



                    while (!m_graph->is_infinite(face_circ_next)) {

                        int nid_of_f1_in_f2 = face_circ_curr->index(face_circ_next);
                        int nid_of_f2_in_f1 = face_circ_next->index(face_circ_curr);

                        face_circ_next->info().f[nid_of_f2_in_f1] += alpha_vis * view_weight;

                        face_circ_curr = face_circ_next;
                        ++face_circ_next;
                    }

                    face_circ_curr->info().s += Options::k_inf;


                    // assign opposite dir
                    Point opposite_dir = CGAL::barycenter(cam_origin, (FT)-0.01, point_pos);
                    Line_face_circulator face_circ_opposite{vh, m_graph.get(), opposite_dir};
                    if (!face_circ_opposite.is_empty() && !m_graph->is_infinite(face_circ_opposite)) {
                        face_circ_opposite->info().t += alpha_vis * view_weight;
                    }
                }
            }
        }

        void computeMaxFlow() {

            std::unordered_map<Face_handle, int> mp_triangulation2seq;
            std::unordered_map<int, Face_handle> mp_seq2triangulation;
            int num_faces = 0;
            for (Face_handle fh : m_graph->all_face_handles()) {
                mp_triangulation2seq[fh] = num_faces;
                mp_seq2triangulation[num_faces] = fh;
                ++num_faces;
            }

            MaxFlow<int, CapacityT> graph(m_graph->number_of_faces());
            // set weights
            constexpr CapacityT maxCap(FLT_MAX*0.0001f);
            for (Face_handle fi : m_graph->all_face_handles()) {
                const auto& fi_info = fi->info();
                int fi_id = mp_triangulation2seq[fi];
                graph.AddNode(fi_id, fi_info.s, std::min(fi_info.t, maxCap));
                for (int i=0; i<3; ++i) {
                    const Face_handle fj(fi->neighbor(i));
                    int fj_id = mp_triangulation2seq[fj];
                    if (fj_id < fi_id) continue;

                    const auto& fj_info = fj->info();
                    const int j(fj->index(fi));
//                    const CapacityT q((1.f - MINF(computePlaneSphereAngle(delaunay, facet_t(ci,i)), computePlaneSphereAngle(delaunay, facet_t(cj,j))))*kQual);
                    const CapacityT q = 0.5;
                    graph.AddEdge(fi_id, fj_id, fi_info.f[i]+q, fj_info.f[j]+q);
                }
            }

            // find graph-cut solution
            const float maxflow = graph.ComputeMaxFlow();

            for (Face_handle fh : m_graph->all_face_handles()) {
                const int fi_id = mp_triangulation2seq[fh];
                if (graph.IsNodeOnSrcSide(fi_id)) {
                    fh->set_in_domain(false);
                }
            }

        }
    };
} // namespace yjl


#endif