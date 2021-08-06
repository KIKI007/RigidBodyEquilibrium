//
// Created by ziqwang on 23.09.20.
//

#include <catch2/catch.hpp>
#include <iostream>
#include "fusion.h"
using namespace mosek::fusion;
using namespace monty;
#include <Eigen/Dense>
TEST_CASE("Mosek Test")
{
    auto h = new_array_ptr<double, 1>({ 6.0, 3.0});

    // Create a model with the name 'lo1'
    Model::t M = new Model("lo1"); auto _M = finally([&]() { M->dispose(); });

    M->setLogHandler([ = ](const std::string & msg) { std::cout << msg << std::flush; } );

    // Create variable 'x' of length 4
    Variable::t x = M->variable("x", 2);

    Variable::t t = M->variable("t", 1);

    // Create constraints
    auto con0 = M->constraint(Expr::sub(x, h), Domain::greaterThan(0.0));

    auto con1 = M->constraint(Expr::vstack(0.5, t, x), Domain::inRotatedQCone());

    // Set the objective function to (c^t * x)
    M->objective("obj", ObjectiveSense::Minimize, t);

    // Solve the problem
    M->solve();

    // Get the solution values
    auto sol = x->level();
    auto dual = con0->dual();

    double x1 = (*sol)[0];
    double x2 = (*sol)[1];
    double l1 = (*dual)[0];
    double l2 = (*dual)[1];

    std::cout << "[dual] = " << (*dual) << "\n";

    std::cout << "[obj] = " << M->primalObjValue() << "\n";

    Eigen::Matrix4d K;
    K << 2, 0, -1, 0,
    0, 2, 0, -1,
    -l1, 0, -x1 + (*h)[0], 0,
    0, -l2, 0, -x2 + (*h)[1];

    Eigen::Vector4d b;
    b << 0, 0, 0, -l2;

    Eigen::Vector4d dx =  K.inverse() * b;
    std::cout << dx[0] * x1 * 2 + dx[1] * x2 * 2 << std::endl;
}