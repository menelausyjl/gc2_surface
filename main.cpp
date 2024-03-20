#include <iostream>
#include "graph.h"
#include <CGAL/point_generators_2.h>
using namespace yjl;

void generateTestCase(Triangulation& triangulation, std::vector<Point>& camera_origins) {
    std::vector<Point> pt2s;

    std::size_t n_points = 100;
    std::size_t n_views = 10;

    camera_origins.reserve(n_views);
    double radius_cam = 2.;
    double radius_points = 1.;
    double dist_thresh = 1.4 * (radius_cam - radius_points) * (radius_cam - radius_points);
    for (std::size_t i = 0; i < n_views; ++i) {
        double angle = (2. * M_PI) * (double)i / (double)n_views;
        double sin_angle = std::sin(angle);
        double cos_angle = std::cos(angle);
        camera_origins.emplace_back(RT(cos_angle * radius_cam), RT(sin_angle * radius_cam), (RT)0);
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

    triangulation.insert(pt2s.begin(), pt2s.end());

    CGAL::Random& rnd = CGAL::get_default_random();
    for (auto vit = triangulation.finite_vertices_begin();
         vit != triangulation.finite_vertices_end();
         ++vit) {
        auto& vertex_info = vit->info();
        auto& point = vit->point();
        for (ViewIdx j = 0; j < camera_origins.size(); ++j) {
            const Point& cam_origin = camera_origins[j];
            FT dist = CGAL::squared_distance(point, cam_origin);
            if (dist > dist_thresh) {
                if (rnd.get_double() < 0.8) {
                    vertex_info[j] += (CapacityT)1;
                }
            }
        }
    }
}

int main() {
    std::cout << "Hello, World!" << std::endl;
    return 0;

    Triangulation triangulation;
    std::vector<Point> cam_origins;
    generateTestCase(triangulation, cam_origins);

    
}
