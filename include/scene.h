#ifndef GCS_SCENE_H
#define GCS_SCENE_H

// #include "types.h"
#include "graph.h"
#include "graph_cut/IBFS.h"

#include <CGAL/Linear_cell_complex_for_combinatorial_map.h>
#include <CGAL/Linear_cell_complex_incremental_builder_3.h>
#include <CGAL/draw_linear_cell_complex.h>
typedef CGAL::Linear_cell_complex_for_combinatorial_map<2, 2> LCC_2;
using Point=LCC_2::Point;

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

        void build() {

            const CapacityT alpha_vis = options.alpha_vis;

            for (int i = 0; i < m_cameras.size(); ++i) {
                const auto& cam = m_cameras[i];
                const Point& cam_origin = cam.origin;
                for (Vertex_handle vh : cam.visible_points) {
                    const Point& point_pos = vh->point();
                    CapacityT view_weight = vh->info()[i];

                    Line_face_circulator face_circ_curr{vh, m_graph.get(), cam_origin};
                    --face_circ_curr;
                    face_circ_curr->info().t += alpha_vis * view_weight;

//                    Line_face_circulator face_circ_end = m_graph->line_walk(cam_origin, point_pos);
                    Line_face_circulator face_circ_next = face_circ_curr;

                    do {

                        int nid_of_f1_in_f2 = face_circ_curr->index(face_circ_next);
                        int nid_of_f2_in_f1 = face_circ_next->index(face_circ_curr);

                        face_circ_next->info().f[nid_of_f2_in_f1] += alpha_vis * view_weight;

                        face_circ_curr = face_circ_next;
                        ++face_circ_next;

                    } while (face_circ_next.handle() != m_graph->infinite_face());

                    face_circ_next->info().s += Options::k_inf;
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
                    const CapacityT q((1.f - MINF(computePlaneSphereAngle(delaunay, facet_t(ci,i)), computePlaneSphereAngle(delaunay, facet_t(cj,j))))*kQual);
                    graph.AddEdge(fi_id, fj_id, fi_info.f[i]+q, fj_info.f[j]+q);
                }
            }

            // find graph-cut solution
            const float maxflow(graph.ComputeMaxFlow());
            // extract surface formed by the facets between inside/outside cells
//            const size_t nEstimatedNumVerts(delaunay.number_of_vertices());
//            std::unordered_map<void*,Mesh::VIndex> mapVertices;
//
//            mesh.vertices.Reserve((Mesh::VIndex)nEstimatedNumVerts);
//            mesh.faces.Reserve((Mesh::FIndex)nEstimatedNumVerts*2);

            std::unordered_map<Vertex_handle, int> mpv_border2seq;
            LCC_2 lcc;
            CGAL::Linear_cell_complex_incremental_builder_3<LCC_2> ib(lcc);

            for (Face_handle fi : m_graph->all_face_handles()) {
                const int fi_id = mp_triangulation2seq[fi];
                for (int i=0; i<3; ++i) {
                    if (m_graph->is_infinite(fi, i)) continue;
                    const Face_handle fj = fi->neighbor(i);
                    const int fj_id = mp_triangulation2seq[fj];
                    if (fi_id < fj_id) continue;
                    const bool fi_side(graph.IsNodeOnSrcSide(fi_id));
                    if (fi_side == graph.IsNodeOnSrcSide(fj_id)) continue;

                    std::set<Vertex_handle> common_verts;
                    for (int vc = 0; vc < 3; ++vc)
                        common_verts.insert(fi->vertex(vc));
                    for (int vc = 0; vc < 3; ++vc) {
                        Vertex_handle vh = fj->vertex(vc);
                        if (common_verts.count(vh)) {
                            ib.add_vertex(vh->point());
                        }
                    }




//                    Mesh::Face& face = mesh.faces.AddEmpty();
//                    const triangle_vhandles_t tri(getTriangle(ci, i));
//                    for (int v=0; v<3; ++v) {
//                        const vertex_handle_t vh(tri.verts[v]);
//                        ASSERT(vh->point() == delaunay.triangle(ci,i)[v]);
//                        const auto pairItID(mapVertices.insert(std::make_pair(vh.for_compact_container(), (Mesh::VIndex)mesh.vertices.GetSize())));
//                        if (pairItID.second)
//                            mesh.vertices.Insert(CGAL2MVS<Mesh::Vertex::Type>(vh->point()));
//                        ASSERT(pairItID.first->second < mesh.vertices.GetSize());
//                        face[v] = pairItID.first->second;
//                    }
//                    // correct face orientation
//                    if (!ciType)
//                        std::swap(face[0], face[2]);
                }
            }

            m_graph->clear();
        }
    };
} // namespace yjl


#endif