//
// Created by Yujie Li on 2024/3/8.
//

#ifndef GCS_TYPES_H
#define GCS_TYPES_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

namespace yjl {
    using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
    using Point_3 = Kernel::Point_3;
    using Point_2 = Kernel::Point_2;
    using Point = Kernel::Point_2;
    using Segment = Kernel::Segment_2;
    using Line = Kernel::Line_2;
    using Triangle = Kernel::Triangle_2;
    using RT = Kernel::RT;
    using FT = Kernel::FT;
}

#endif //GCS_TYPES_H
