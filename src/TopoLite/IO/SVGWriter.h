//
// Created by ziqwang on 14.10.20.
//

#ifndef TOPOLITE_SVGWRITER_H
#define TOPOLITE_SVGWRITER_H

#include "Utility/simple_svg.hpp"
#include "Mesh/Polygon.h"
class SVGWriter{
public:

    const double unit_to_mm = 1000;

public:

    template<typename Object>
    void writeToFile(std::string folder,
                     const vector<Object> &parts,
                     const vector<Object> &boundaries){
        for(int id = 0; id < parts.size(); id++){
            std::string filename = folder + "/" + "part_" + to_string(id) + ".svg";
            writeObjectToFile(filename, parts[id]);
        }

        for(int id = 0; id < boundaries.size(); id++){
            std::string filename = folder + "/" + "bdry_" + to_string(id) + ".svg";
            writeObjectToFile(filename, boundaries[id]);
        }
    }

    template<typename Scalar>
    void writeObjectToFile(std::string filename, shared_ptr<_Polygon<Scalar>> poly){
        Box<double> bbx = poly->bbox();

        svg::Dimensions dimensions(0);
        svg::Document doc(filename, svg::Layout(dimensions, svg::Layout::BottomLeft));

// Long notation.  Local variable is created, children are added to varaible.
        svg::Polygon svg_poly(svg::Stroke(1, svg::Color::Red));
        for(int id = 0; id < poly->size(); id++){
            auto pA = poly->pos(id);
            svg_poly << svg::Point(mm(pA[0]), mm(pA[1]));
        }
        doc << svg_poly;

        doc.save();
    }

    template<typename Scalar>
    void writeObjectToFile(std::string filename, vector<vector<shared_ptr<_Polygon<Scalar>>>>& polys){
        svg::Dimensions dimensions(0);
        svg::Document doc(filename, svg::Layout(dimensions, svg::Layout::BottomLeft));

        for(int id = 0; id < polys.size(); id++)
        {
            svg::PolygonGroup group(svg::Stroke(1, svg::Color::Red));
            for(int jd = 0; jd < polys[id].size(); jd++)
            {
                svg::Polygon svg_poly(svg::Stroke(1, svg::Color::Red));
                shared_ptr<_Polygon<double>> poly = polys[id][jd];
                for(int kd = 0; kd < poly->size(); kd++)
                {
                    auto pA = poly->pos(kd);
                    svg_poly << svg::Point(mm(pA[0]), mm(pA[1]));
                }
                group << svg_poly;
            }
            doc << group;
        }
        doc.save();
    }

    template<typename Scalar>
    void writeObjectToFile(std::string filename, const vector<Line<Scalar>> &lines){
        svg::Dimensions dimensions(0);
        svg::Document doc(filename, svg::Layout(dimensions, svg::Layout::BottomLeft));

// Long notation.  Local variable is created, children are added to varaible.
        svg::LineGroup group(svg::Stroke(1, svg::Color::Red));
        for(int id = 0; id < lines.size(); id++){
            svg::Point p0 =  svg::Point(mm(lines[id].point1[0]), mm(lines[id].point1[1]));
            svg::Point p1 =  svg::Point(mm(lines[id].point2[0]), mm(lines[id].point2[1]));
            svg::Line line(p0, p1);
            group << line;
        }
        doc << group;
        doc.save();
    }

    template<typename Scalar>
    void writeObjectToFile(std::string filename, const vector<vector<Line<Scalar>>> &lines){

        svg::Dimensions dimensions(0);
        svg::Document doc(filename, svg::Layout(dimensions, svg::Layout::BottomLeft));

// Long notation.  Local variable is created, children are added to varaible.
        for(const vector<Line<Scalar>> &ls: lines){
            svg::LineGroup group(svg::Stroke(1, svg::Color::Red));
            for(int id = 0; id < ls.size(); id++){
                svg::Point p0 =  svg::Point(mm(ls[id].point1[0]), mm(ls[id].point1[1]));
                svg::Point p1 =  svg::Point(mm(ls[id].point2[0]), mm(ls[id].point2[1]));
                svg::Line line(p0, p1);
                group << line;
            }
            doc << group;
        }

        doc.save();
    }

    template<typename Scalar>
    void writeCirclesToFile(std::string filename, const vector<Matrix<Scalar, 2, 1>> &points, Scalar radius){

        svg::Dimensions dimensions(0);
        svg::Document doc(filename, svg::Layout(dimensions, svg::Layout::BottomLeft));

        int size = mm(radius);
// Long notation.  Local variable is created, children are added to varaible.
        for(const Eigen::Vector2d pt: points)
        {
            svg::Fill fill(svg::Color::Yellow);
            svg::Stroke stock(1);
            svg::Point svg_pt = svg::Point(mm(pt[0]), mm(pt[1]));
            svg::Circle cirlce = svg::Circle(svg_pt, size, fill, stock);
            doc << cirlce;
        }

        doc.save();
    }


    double mm(double x){
        return x * unit_to_mm;
    }

public:

};

#endif //TOPOLITE_SVGWRITER_H
