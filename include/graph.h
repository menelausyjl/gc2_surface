//
// Created by Yujie Li on 2024/3/8.
//

#ifndef GCS_GRAPH_H
#define GCS_GRAPH_H

#include "types.h"
#include "domain_face_base_with_info.h"
#include <CGAL/Delaunay_triangulation_2.h>
//#include <CGAL/Constrained_Delaunay_triangulation_2.h>
//#include <CGAL/Constrained_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>


namespace yjl {
    using ViewIdx = unsigned short;
    using CapacityT = float;

    using VertexInfo = std::map<ViewIdx, CapacityT>;

    struct FaceInfo {
        typedef CapacityT Type;
        Type f[3] {}; // edges' weight from the face outwards
        Type s {}; // face's weight towards s-source
        Type t {}; // face's weight towards t-sink
        inline const Type* ptr() const { return f; }
        inline Type* ptr() { return f; }
    };

    typedef CGAL::Triangulation_vertex_base_with_info_2<VertexInfo, Kernel> VertexBaseT;
    typedef CGAL::Triangulation_face_base_with_domain_and_info_2<FaceInfo, Kernel> FaceBaseT;
    typedef CGAL::Triangulation_data_structure_2<VertexBaseT, FaceBaseT> TriangulationDataStructure;

    using Triangulation = CGAL::Delaunay_triangulation_2<Kernel, TriangulationDataStructure>;

    typedef Triangulation::Face_handle    Face_handle;
    typedef Triangulation::Vertex_handle  Vertex_handle;
    typedef Triangulation::Locate_type    Locate_type;
    typedef Triangulation::Edge             Edge;
    using Line_face_circulator = Triangulation::Line_face_circulator;

    inline Edge getCommonEdge(Face_handle f1, Face_handle f2) {
        if (!f1->has_neighbor(f2) || f2->has_neighbor(f1)) {
            return {};
        }
        return {f1, f1->index(f2)};
    }

    inline FT computePlaneSphereAngle(Face_handle fh, int i) {
        // TODO: implement this

        return {};
    }

    
    struct Camera {
        Point origin;
        std::vector<Vertex_handle> visible_points;

        explicit Camera(const Point& o) : origin(o) {}
    };
}

#endif //GCS_GRAPH_H
