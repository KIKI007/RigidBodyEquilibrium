#include <catch2/catch.hpp>
#include <stdio.h>
#include "Stability/Solver/Equilibrium/EquilibriumFrictionLessSolver_Mosek.h"

TEST_CASE("two squares stacking together: stability"){
    shared_ptr<InputVarList> varList;
    varList = make_shared<InputVarList>();
    InitVar(varList.get());

    shared_ptr<ContactGraphBase<double>> graph = make_shared<ContactGraphBase<double>>(varList);

    shared_ptr<ContactGraphNode<double>> node0;
    node0 = make_shared<ContactGraphNode<double>>(true, Vector3d(0, 0, 0), Vector3d(0, 0, 0), 4*2);

    shared_ptr<ContactGraphNode<double>> node1;
    node1 = make_shared<ContactGraphNode<double>>(false, Vector3d(0, 2, 0), Vector3d(0, 2, 0), 4*2);

    graph->addNode(node0);
    graph->addNode(node1);

    shared_ptr<_Polygon<double>> polygon = make_shared<_Polygon<double>>();
    polygon->push_back(Vector3d(-2, 1, 0));
    polygon->push_back(Vector3d(2, 1, 0));

    shared_ptr<ContactGraphEdgeBase<double>> edge;
    edge = make_shared<ContactGraphEdgeBase<double>>(polygon, Vector3d(0, 1, 0));

    EquilibriumFrictionLessSolver_Mosek<2> solver(graph);
    shared_ptr<EquilibriumFrictionLessSolver<2>::StaticData> data;
    shared_ptr<EquilibriumFrictionLessSolver<2>::KinematicData> kdata;

    Eigen::MatrixXd equ_mat;
    Eigen::VectorXd force;

    SECTION("output the matrix"){
        graph->addContact(node0, node1, edge);
        graph->finalize();
        solver.computeEquilibriumMatrix(equ_mat, false, true);
        solver.getForces(Vector3d(0, -1, 0), force, true);
        std::cout << "square:\t" << std::endl;
        std::cout << equ_mat << std::endl;
        std::cout << force << std::endl << std::endl;
    }

    SECTION("check"){
        node1->centerofmass = Vector3d(3.01, 1,0);
        graph->addContact(node0, node1, edge);
        graph->finalize();
        REQUIRE(solver.isEquilibrium_Constraints(Vector3d(0, -1, 0), data) == false);
    }
}

TEST_CASE("two cubes stacking together: stability"){

    shared_ptr<InputVarList> varList;
    varList = make_shared<InputVarList>();
    InitVar(varList.get());

    shared_ptr<ContactGraphBase<double>> graph = make_shared<ContactGraphBase<double>>(varList);

    shared_ptr<ContactGraphNode<double>> node0;
    node0 = make_shared<ContactGraphNode<double>>(true, Vector3d(0, 0, 1), Vector3d(0, 0, 1), 4*2*2);

    shared_ptr<ContactGraphNode<double>> node1;
    node1 = make_shared<ContactGraphNode<double>>(false, Vector3d(1, 0, 3), Vector3d(1, 0, 3), 4*2*2);

    graph->addNode(node0);
    graph->addNode(node1);

    shared_ptr<_Polygon<double>> polygon = make_shared<_Polygon<double>>();
    polygon->push_back(Vector3d(-2, -1, 2));
    polygon->push_back(Vector3d(2, -1, 2));
    polygon->push_back(Vector3d(2, 1, 2));
    polygon->push_back(Vector3d(-2, 1, 2));

    shared_ptr<ContactGraphEdgeBase<double>> edge;
    edge = make_shared<ContactGraphEdgeBase<double>>(polygon, Vector3d(0, 0, 1));


    Eigen::MatrixXd equ_mat;
    Eigen::VectorXd force;

    EquilibriumFrictionLessSolver_Mosek<3> solver(graph);
    shared_ptr<EquilibriumFrictionLessSolver<3>::StaticData> data;
    shared_ptr<EquilibriumFrictionLessSolver<3>::KinematicData> kdata;


    SECTION("output the matrix"){
        std::cout << "cube:\t" << std::endl;
        node0->isBoundary = false;
        graph->addContact(node0, node1, edge);
        graph->finalize();
        solver.computeEquilibriumMatrix(equ_mat, false, true);
        std::cout << equ_mat.transpose() << std::endl << std::endl;
    }

    SECTION("Test Equilibrium"){
        graph->addContact(node0, node1, edge);
        graph->finalize();
        REQUIRE(solver.isEquilibrium_Variables(Vector3d(0, 0, -1), data) == true);
    }

    SECTION("Test Sequential Equilibrium"){
        graph->addContact(node0, node1, edge);
        graph->finalize();
        for(int id = 0; id < 100; id++){
            node1->centerofmass = Vector3d(0.03 * id, 0, 3);
            solver.silence = true;
            solver.isEquilibrium_Dual(Vector3d(0, 0, -1), kdata);
            solver.isEquilibrium_Constraints(Vector3d(0, 0, -1), data);
            //std::cout << id * 0.03 << ":\t" << data->energy << ", " << kdata->energy << std::endl;
            REQUIRE(std::abs(data->energy - kdata->energy) == Approx(0.0).margin(1E-6));
        }
    }
}
