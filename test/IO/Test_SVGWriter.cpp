//
// Created by ziqwang on 14.10.20.
//

#include "IO/SVGWriter.h"
#include "catch2/catch.hpp"

TEST_CASE("SVGWriter"){
    SVGWriter writer;

    shared_ptr<_Polygon<double>> poly = make_shared<_Polygon<double>>();
    poly->push_back(Vector3d(0, 0, 0));
    poly->push_back(Vector3d(1, 0, 0));
    poly->push_back(Vector3d(1, 1, 0));
    poly->push_back(Vector3d(0, 1, 0));

    vector<Line<double>> lines;
    lines.push_back(Line<double>(Vector3d(0, 0, 0), Vector3d(1, 0, 0)));

    writer.writeObjectToFile("square.svg", poly);
    writer.writeObjectToFile("lines.svg", lines);

}