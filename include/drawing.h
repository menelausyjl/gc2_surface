//
// Created by Yujie Li on 2024/3/27.
//

#ifndef GRAPH_CUT_2D_DRAWING_H
#define GRAPH_CUT_2D_DRAWING_H

// Copyright(c) 2018  INRIA Sophia-Antipolis (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/v5.6.1/Triangulation_2/include/CGAL/draw_triangulation_2.h $
// $Id: draw_triangulation_2.h 4547818 2022-11-15T13:39:40+01:00 albert-github
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s)     : Guillaume Damiand <guillaume.damiand@liris.cnrs.fr>


#include <CGAL/draw_triangulation_2.h>

// Specialization of draw function.
namespace yjl {

    struct DomainColorFunctorT2
    {
        template<typename T2>
        static CGAL::IO::Color run(const T2&,
                                   const typename T2::Finite_faces_iterator fh)
        {
            CGAL::Random random((unsigned int)(std::size_t)(&*fh));
            static CGAL::IO::Color c_out = get_random_color(random);
            static CGAL::IO::Color c_in = get_random_color(random);
            return fh->is_in_domain() ? c_in : c_out;
        }
    };

    template<class Gt, class Tds>
    void drawWithDomain(const CGAL::Triangulation_2<Gt, Tds>& at2)
    {
        const char* title="Triangulation_2 Basic Viewer";
        bool nofill=false;
#if defined(CGAL_TEST_SUITE)
        bool cgal_test_suite=true;
#else
        bool cgal_test_suite=qEnvironmentVariableIsSet("CGAL_TEST_SUITE");
#endif

        if (!cgal_test_suite)
        {
            CGAL::Qt::init_ogl_context(4,3);
            int argc=1;
            const char* argv[2]={"t2_viewer", nullptr};
            QApplication app(argc,const_cast<char**>(argv));
            DomainColorFunctorT2 fcolor;
            CGAL::SimpleTriangulation2ViewerQt<CGAL::Triangulation_2<Gt, Tds>, DomainColorFunctorT2>
                    mainwindow(app.activeWindow(), at2, title, nofill, fcolor);
            mainwindow.show();
            app.exec();
        }
    }
}
#endif //GRAPH_CUT_2D_DRAWING_H
