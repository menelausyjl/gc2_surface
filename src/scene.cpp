//
// Created by Yujie Li on 2024/4/5.
//
#include "scene.h"

namespace yjl {

    Scene::Scene() {
        m_graph = std::make_shared<Triangulation>();
    }

    void Scene::build(CapacityT thickness_sigma) {

        const CapacityT alpha_vis = options.alpha_vis;

        FT median_thickness_sqr = calculateMedianThickness();

        const FT inv2_sigma_sqr = (FT)0.5 / (median_thickness_sqr * options.thickness_sigma * options.thickness_sigma);

        const auto& distWeight = [=](const FT dist_sqr)->CapacityT {
            return (CapacityT)((FT)1 - std::exp(-dist_sqr * inv2_sigma_sqr));
//            return (CapacityT)1;
        };

//        for (Face_handle fi : m_graph->all_face_handles()) {
//            if (m_graph->is_infinite(fi)) {
//                fi->info().s = Options::k_inf;
//            }
//        }

        for (int i = 0; i < m_cameras.size(); ++i) {

            const auto& cam = m_cameras[i];
            const Point& cam_origin = cam.origin;
            Face_handle cam_face = m_graph->locate(cam_origin);
            assert(m_graph->is_infinite(cam_face));
//                cam_face->info().s += Options::k_inf;

            for (Vertex_handle vh : cam.visible_points) {
//                    std::cout << vh->point() << " ... \n";
                const Point& point_pos = vh->point();
                CapacityT view_weight = vh->info()[i];

                Ray ray{cam_origin, point_pos};

                // assign opposite dir
                Point opposite_dir = CGAL::barycenter(cam_origin, (FT)-0.000001, point_pos);
                Face_handle fh_oppo = m_graph->locate(opposite_dir);
                assert(fh_oppo->has_vertex(vh));
                if (!m_graph->is_infinite(fh_oppo)) {
                    FT dist_sqr = CGAL::squared_distance(cam_origin, point_pos);
                    fh_oppo->info().t += distWeight(dist_sqr) * view_weight;
//                    fh_oppo->info().t += Options::k_inf * 10.f;
                }

                Line_face_circulator face_circ_curr{vh, m_graph.get(), cam_origin};
                if (face_circ_curr.is_empty() || m_graph->is_infinite(face_circ_curr)) {
                    continue;
                }
                assert(face_circ_curr != fh_oppo);
                Line_face_circulator face_circ_next = face_circ_curr;
                ++face_circ_next;

                while (!m_graph->is_infinite(face_circ_curr)) {

//                    int nid_of_f1_in_f2 = face_circ_curr->index(face_circ_next);
                    int nid_of_f2_in_f1 = face_circ_next->index(face_circ_curr);

                    Segment segment{face_circ_next->vertex(CGAL::Triangulation_cw_ccw_2::cw(nid_of_f2_in_f1))->point(),
                                    face_circ_next->vertex(CGAL::Triangulation_cw_ccw_2::ccw(nid_of_f2_in_f1))->point()};

                    Point intersection = rayEdgeIntersect(ray, segment);
                    assert(intersection != CGAL::ORIGIN);

                    FT dist_sqr = CGAL::squared_distance(cam_origin, intersection);
                    std::cout << distWeight(dist_sqr) << std::endl;

                    face_circ_next->info().f[nid_of_f2_in_f1] += distWeight(dist_sqr) * view_weight;

//                    ++face_circ_curr;
//                    assert(face_circ_curr == face_circ_next);
                    face_circ_curr = face_circ_next;
                    ++face_circ_next;
                }

                face_circ_curr->info().s += Options::k_inf;

//                for (int j = 0; j < 3; ++j) {
//                    Face_handle incident_infinite_fh = face_circ_curr->neighbor(j);
//                    if (m_graph->is_infinite(incident_infinite_fh)) {
//                        int nid_of_f2_in_f1 = incident_infinite_fh->index(face_circ_curr);
//                        int inf_id = incident_infinite_fh->index(m_graph->infinite_vertex());
////                        std::cout << nid_of_f2_in_f1 << "..." << inf_id << std::endl;
//                        incident_infinite_fh->info().f[nid_of_f2_in_f1] += alpha_vis * view_weight;
//                        break;
//                    }
//                }

//                    Line_face_circulator face_circ_opposite{vh, m_graph.get(), opposite_dir};
//                    if (!face_circ_opposite.is_empty() && !m_graph->is_infinite(face_circ_opposite)) {
//                        face_circ_opposite->info().t += alpha_vis * view_weight;
//                    }
            }
        }
    }

    void Scene::buildForward() {

        const CapacityT alpha_vis = options.alpha_vis;

        for (int i = 0; i < m_cameras.size(); ++i) {
//                std::cout << i << "///\n";

            const auto& cam = m_cameras[i];
            const Point& cam_origin = cam.origin;
            Face_handle cam_face = m_graph->locate(cam_origin);
            assert(m_graph->is_infinite(cam_face));
//                cam_face->info().s += Options::k_inf;

            for (Vertex_handle vh : cam.visible_points) {
//                    std::cout << vh->point() << " ... \n";
                const Point& point_pos = vh->point();
                CapacityT view_weight = vh->info()[i];

                Line_face_circulator face_circ_curr{vh, m_graph.get(), cam_origin};
                auto fc = m_graph->line_walk(cam_origin, point_pos, cam_face);
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

//                    assert(face_circ_next == cam_face);
                face_circ_next->info().s += Options::k_inf;

                // assign opposite dir
                Point opposite_dir = CGAL::barycenter(cam_origin, (FT)-0.00001, point_pos);
                Line_face_circulator face_circ_opposite{vh, m_graph.get(), opposite_dir};
                assert(face_circ_opposite->has_vertex(vh));
                if (!face_circ_opposite.is_empty() && !m_graph->is_infinite(face_circ_opposite)) {
                    face_circ_opposite->info().t += alpha_vis * view_weight;
                }
            }
        }
    }

    void Scene::computeMaxFlow() {

        std::unordered_map<Face_handle, int> mp_triangulation2seq;
        std::unordered_map<int, Face_handle> mp_seq2triangulation;
        int num_faces = 0;
        int num_infinite_faces = 0;
        for (Face_handle fh : m_graph->all_face_handles()) {
            if (m_graph->is_infinite(fh)) {
                ++num_infinite_faces;
            }
            mp_triangulation2seq[fh] = num_faces;
            mp_seq2triangulation[num_faces] = fh;
            ++num_faces;
        }

        MaxFlow<int, CapacityT> graph(num_faces);
        // set weights
        constexpr CapacityT maxCap(FLT_MAX*0.0001f);
        for (Face_handle fi : m_graph->all_face_handles()) {
            const auto& fi_info = fi->info();
            int fi_id = mp_triangulation2seq[fi];
//            if (m_graph->is_infinite(fi)) {
//                std::cerr << "!!!\n";
//            }
            graph.AddNode(fi_id, fi_info.s, std::min(fi_info.t, maxCap));

//            std::cout << fi_id << ": " << fi_info.s << " << " << fi_info.t << std::endl;
            for (int i=0; i<3; ++i) {
                const Face_handle fj = fi->neighbor(i);
                int fj_id = mp_triangulation2seq[fj];
                if (fj_id < fi_id) continue;

                const auto& fj_info = fj->info();
                const int j = fj->index(fi);
                const CapacityT q = (1. - std::min(computePlaneSphereAngle(fi, i), computePlaneSphereAngle(fj, j))) * 0.3;
//                const CapacityT q = 0.5;
//                std::cout << fi_id << ", " << fj_id << ", " << fi_info.f[i] << ", " << fj_info.f[j] << std::endl;
                graph.AddEdge(fi_id, fj_id, fi_info.f[i]+q, fj_info.f[j]+q);
            }
        }

        // find graph-cut solution
        const float maxflow = graph.ComputeMaxFlow();
        std::cout << "Max flow: " << maxflow << std::endl;

        for (Face_handle fh : m_graph->all_face_handles()) {
            const int fi_id = mp_triangulation2seq[fh];
            if (graph.IsNodeOnSrcSide(fi_id)) {
                fh->set_in_domain(false);
            }
        }

    }

    FT Scene::calculateMedianThickness() const {
        std::vector<FT> thicknesses;
        for (auto ei : m_graph->finite_edges()) {
            Face_handle fi = ei.first;
            int j = ei.second;
            Face_handle fj = fi->neighbor(j);
            int i = fj->index(fi);
            FT dist_sqr = CGAL::squared_distance(fi->vertex(j)->point(), fj->vertex(i)->point());
            thicknesses.emplace_back(dist_sqr);
        }
        std::nth_element(thicknesses.begin(), thicknesses.begin() + thicknesses.size() / 2, thicknesses.end());
        return thicknesses[thicknesses.size() / 2];
    }
}