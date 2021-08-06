//
// Created by ziqwang on 14.08.20.
//

#include <catch2/catch.hpp>
#include "Stability/Solver/Equilibrium/EquilibriumFrictionLessSolver.h"
#include "Stability/Solver/Interlocking/InterlockingSolver.h"

TEST_CASE("two cubes stacking together: matrix"){

    shared_ptr<InputVarList> varList;
    varList = make_shared<InputVarList>();
    InitVar(varList.get());

    shared_ptr<ContactGraphBase<double>> graph = make_shared<ContactGraphBase<double>>(varList);

    shared_ptr<ContactGraphNode<double>> node0;
    node0 = make_shared<ContactGraphNode<double>>(true, Vector3d(0, 0, 1), Vector3d(0, 0, 1), 4*2*2);

    shared_ptr<ContactGraphNode<double>> node1;
    node1 = make_shared<ContactGraphNode<double>>(false, Vector3d(0, 0, 3), Vector3d(0, 0, 3), 4*2*2);

    graph->addNode(node0);
    graph->addNode(node1);

    shared_ptr<_Polygon<double>> polygon = make_shared<_Polygon<double>>();
    polygon->push_back(Vector3d(-2, -1, 2));
    polygon->push_back(Vector3d(2, -1, 2));
    polygon->push_back(Vector3d(2, 1, 2));
    polygon->push_back(Vector3d(-2, 1, 2));

    shared_ptr<ContactGraphEdgeBase<double>> edge;
    edge = make_shared<ContactGraphEdgeBase<double>>(polygon, Vector3d(0, 0, 1));

    graph->addContact(node0, node1, edge);

    graph->finalize();

    EquilibriumFrictionLessSolver<3> solver(graph);
    Eigen::MatrixXd equ_mat;
    solver.computeEquilibriumMatrix(equ_mat, false);

    Eigen::MatrixXd int_mat;
    InterlockingSolver<double> interlock_solver(graph);
    interlock_solver.computeRotationalInterlockingMatrixDense(int_mat);

    //transpose of interlocking matrix must be equal to the equilibrium matrix
    REQUIRE((equ_mat - int_mat.transpose()).norm() == Approx(0).margin(1e-5));
}

