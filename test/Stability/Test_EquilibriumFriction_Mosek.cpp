//
// Created by ziqwang on 09.08.21.
//
#include "Stability/Solver/Equilibrium/EquilibriumFrictionSolver_Mosek.h"
#include "catch2/catch.hpp"

TEST_CASE("two cubes stacking problem: stability with friction")
{
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

    graph->addContact(node0, node1, edge);
    graph->finalize();

    Eigen::MatrixXd equ_mat, fr_mat;
    Eigen::VectorXd force;

    EquilibriumFrictionSolver_Mosek solver(graph);

    shared_ptr<EquilibriumFrictionSolver_Mosek::StaticData> data;
    std::cout << "Is Equilibrium:\t" << solver.isEquilibrium(Vector3d(0, 0, 1), data) << std::endl;

    if(data){
        std::cout << data->energy << std::endl;
        for(int id = 0; id < data->contact_points.size(); id++){
            std::cout << "contact point:\t" << data->contact_points[id].transpose() << std::endl;
            std::cout << "compression:\t" << data->compression_forces[id].transpose() << std::endl;
            std::cout << "tension:\t" << data->tension_forces[id].transpose() << std::endl;
            std::cout << "friction:\t" << data->friction_forces[id].transpose() << std::endl;
            std::cout << "\n";
        }
    }
}