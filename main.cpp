#include <iostream>
#include "scene.h"
#include <CGAL/point_generators_2.h>
#include <CGAL/draw_linear_cell_complex.h>
using namespace yjl;

void generateTestCase(yjl::Scene& scene) {
    std::vector<Point> pt2s;

    std::size_t n_points = 100;
    std::size_t n_views = 10;

    scene.m_cameras.reserve(n_views);
    double radius_cam = 2.;
    double radius_points = 1.;
    double dist_thresh = 1.4 * (radius_cam - radius_points) * (radius_cam - radius_points);
    for (std::size_t i = 0; i < n_views; ++i) {
        double angle = (2. * M_PI) * (double)i / (double)n_views;
        double sin_angle = std::sin(angle);
        double cos_angle = std::cos(angle);
        scene.m_cameras.emplace_back(Point{RT(cos_angle * radius_cam), RT(sin_angle * radius_cam)});
    }

    typedef CGAL::Random_points_on_circle_2<
            Point,
            CGAL::Creator_uniform_2<RT, Point>>
            Point_generator;

    Point_generator pg{radius_points};

//    CGAL::random_convex_set_2(
//            500,
//            std::back_inserter(pt2s),
//            Point_generator( 0.5));

    std::copy_n(pg, n_points, std::back_inserter(pt2s));

    CGAL::perturb_points_2(pt2s.begin(), pt2s.end(), 0.25);

    scene.m_graph->insert(pt2s.begin(), pt2s.end());

    CGAL::Random& rnd = CGAL::get_default_random();
    for (auto vit = scene.m_graph->finite_vertices_begin();
         vit != scene.m_graph->finite_vertices_end();
         ++vit) {
        auto& vertex_info = vit->info();
        auto& point = vit->point();
        for (ViewIdx j = 0; j < scene.m_cameras.size(); ++j) {
            Camera& camera = scene.m_cameras[j];
            FT dist = CGAL::squared_distance(point, camera.origin);
            if (dist > dist_thresh) {
                if (rnd.get_double() < 0.8) {
                    vertex_info[j] += (CapacityT)1;
                    camera.visible_points.emplace_back(vit);
                }
            }
        }
    }
}

int main() {
    std::cout << "Hello, World!" << std::endl;
    return 0;

    yjl::Scene scene;
    generateTestCase(scene);

    
}
