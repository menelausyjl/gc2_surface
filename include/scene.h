#ifndef GCS_SCENE_H
#define GCS_SCENE_H

// #include "types.h"
#include "graph.h"

namespace yjl
{
    class Scene {
    public:
        Triangulation m_graph;
        std::vector<Camera> m_cameras;

        void build() {
            for (const auto& cam : m_cameras) {
                const Point& cam_origin = cam.origin;
                for (Vertex_handle vh : cam.visible_points) {
                    const Point& point_pos = vh->point();
                    Line_face_circulator face_circ_end = m_graph.line_walk(cam_origin, point_pos);
                    Line_face_circulator face_circ = face_circ_end;
                    do {
                        std::cout << *face_circ << std::endl;
                    } while (face_circ != face_circ_end);
                }
            }
        }
    };
} // namespace yjl


#endif