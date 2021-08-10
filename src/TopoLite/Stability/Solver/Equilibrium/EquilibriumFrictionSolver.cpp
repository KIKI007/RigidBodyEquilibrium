//
// Created by ziqwang on 14.08.20.
//

#include "EquilibriumFrictionSolver.h"

void EquilibriumFrictionSolver::get_force_from_norm_fric(Vector3   n,
                                                              Vector3   u,
                                                              Vector3   v,
                                                              RowVector4 &fkx,
                                                              RowVector4 &fky,
                                                              RowVector4 &fkz){
    Vector3 x_comp(1, 0 ,0);
    Vector3 y_comp(0, 1 ,0);
    Vector3 z_comp(0, 0 ,1);
    fkx = RowVector4(n.dot(x_comp), -n.dot(x_comp), u.dot(x_comp), v.dot(x_comp));
    fky = RowVector4(n.dot(y_comp), -n.dot(y_comp), u.dot(y_comp), v.dot(y_comp));
    fkz = RowVector4(n.dot(z_comp), -n.dot(z_comp), u.dot(z_comp), v.dot(z_comp));
}

void EquilibriumFrictionSolver::get_moment_from_norm_fric_vertex(Vector3 n,
                                                                      Vector3 u,
                                                                      Vector3 v,
                                                                      Vector3 r,
                                                                      RowVector4 &mx,
                                                                      RowVector4 &my,
                                                                      RowVector4 &mz)
{

    Vector3 x_comp(1, 0 ,0);
    Vector3 y_comp(0, 1 ,0);
    Vector3 z_comp(0, 0 ,1);
    Vector3 m_normal = -r.cross(n);
    Vector3 m_u_fric = -r.cross(u);
    Vector3 m_v_fric = -r.cross(v);
    mx = RowVector4(m_normal.dot(x_comp), -m_normal.dot(x_comp), m_u_fric.dot(x_comp), m_v_fric.dot(x_comp));
    my = RowVector4(m_normal.dot(y_comp), -m_normal.dot(y_comp), m_u_fric.dot(y_comp), m_v_fric.dot(y_comp));
    mz = RowVector4(m_normal.dot(z_comp), -m_normal.dot(z_comp), m_u_fric.dot(z_comp), m_v_fric.dot(z_comp));
}

void EquilibriumFrictionSolver::get_A_j_k(int partID,
                                               int edgeID,
                                               Eigen::MatrixXd &Ajk) {
    // 1.0 get interface and part
    shared_ptr<ContactGraphEdgeBase<double>> edge = graph->edges[edgeID];
    shared_ptr<ContactGraphNode<double>> part = graph->nodes[partID];

    // 2.0 initialize the matrix Ajk
    int num_vks = edge->size();
    Ajk = Eigen::MatrixXd(6, 4 * num_vks);
    for(size_t id = 0; id < edge->points.size(); id++)
    {
        // 3.0 filling the force fx, fy, fz, term
        Matrix<double, 3, 1> normal, u_fric, v_fric;
        RowVector4 fkx, fky, fkz;
        edge->get_norm_fric_for_block(partID, id, normal, u_fric, v_fric);
        get_force_from_norm_fric(normal, u_fric, v_fric, fkx, fky, fkz);
        Ajk.block(0, 4 * id, 1, 4 ) = fkx;
        Ajk.block(1, 4 * id, 1, 4 ) = fky;
        Ajk.block(2, 4 * id, 1, 4 ) = fkz;

        // 4.0 filling the torque mx, my, mz term
        Vector3 r = (edge->points[id] - part->centroid);
        RowVector4 mx, my, mz;
        get_moment_from_norm_fric_vertex(normal, u_fric, v_fric, r, mx, my, mz);
        Ajk.block(3, 4 * id, 1, 4) = mx;
        Ajk.block(4, 4 * id, 1, 4) = my;
        Ajk.block(5, 4 * id, 1, 4) = mz;
    }
}
void EquilibriumFrictionSolver::computeEquilibriumMatrix(vector<EigenTriple> &tripleList, int &nrow, int &ncol) {
    // 1.0 init the matrix
    vector<int> row_start_index;
    int start_index = 0;
    for (shared_ptr<ContactGraphEdgeBase<double>> edge: graph->edges)
    {
        row_start_index.push_back(start_index);
        start_index += edge->size() * 4;
    }
    tripleList.clear();
    nrow = 6 * graph->dynamic_nodes.size();
    ncol = start_index;

    // 2.0 build matrix
    for (size_t id = 0; id < graph->edges.size(); id++) {
        shared_ptr<ContactGraphEdgeBase<double>> edge = graph->edges[id];
        shared_ptr<ContactGraphNode<double>> partA = graph->nodes[edge->partIDA];
        shared_ptr<ContactGraphNode<double>> partB = graph->nodes[edge->partIDB];
        int dyn_partIDA = partA->dynamicID;
        int dyn_partIDB = partB->dynamicID;
        int num_vk = edge->size();
        if (!partA->isBoundary)
        {
            Eigen::MatrixXd Ajk;
            get_A_j_k(partA->staticID, id, Ajk);
            appendToTripleList(6 * dyn_partIDA, row_start_index[id], Ajk, tripleList);
        }
        if (!partB->isBoundary) {
            Eigen::MatrixXd Ajk;
            get_A_j_k(partB->staticID, id, Ajk);
            appendToTripleList(6 * dyn_partIDB, row_start_index[id], Ajk, tripleList);
        }
    }

    return;
}

void EquilibriumFrictionSolver::computeFrictionMatrix(vector<EigenTriple> &tripleList, int &nrow, int &ncol)
{
    int rowIndex = 0;
    int colIndex = 0;

    double alpha = getVarList()->getFloat("friction_coeff");

    for (size_t id = 0; id < graph->edges.size(); id++)
    {
        shared_ptr<ContactGraphEdgeBase<double>> edge = graph->edges[id];
        for(size_t jd = 0; jd < edge->size(); jd++)
        {
            int index_compression = colIndex;
            int index_tension = colIndex + 1;
            int index_ufriction = colIndex + 2;
            int index_vfriction = colIndex + 3;

            //ufriction - alpha * compression + alpha * tension <= 0
            tripleList.push_back(Eigen::Triplet<double>(rowIndex, index_ufriction, 1.0));
            tripleList.push_back(Eigen::Triplet<double>(rowIndex, index_compression, -alpha));
            //tripleList.push_back(Eigen::Triplet<double>(rowIndex, index_tension, alpha));
            rowIndex ++;

            //-ufriction - alpha * compression + alpha * tension <= 0
            tripleList.push_back(Eigen::Triplet<double>(rowIndex, index_ufriction, -1.0));
            tripleList.push_back(Eigen::Triplet<double>(rowIndex, index_compression, -alpha));
            //tripleList.push_back(Eigen::Triplet<double>(rowIndex, index_tension, alpha));
            rowIndex ++;

            //vfriction - alpha * compression + alpha * tension <= 0
            tripleList.push_back(Eigen::Triplet<double>(rowIndex, index_vfriction, 1.0));
            tripleList.push_back(Eigen::Triplet<double>(rowIndex, index_compression, -alpha));
            //tripleList.push_back(Eigen::Triplet<double>(rowIndex, index_tension, alpha));
            rowIndex ++;

            //-vfriction - alpha * compression + alpha * tension <= 0
            tripleList.push_back(Eigen::Triplet<double>(rowIndex, index_vfriction, -1.0));
            tripleList.push_back(Eigen::Triplet<double>(rowIndex, index_compression, -alpha));
            //tripleList.push_back(Eigen::Triplet<double>(rowIndex, index_tension, alpha));
            rowIndex ++;

            colIndex += 4;
        }
    }

    nrow = rowIndex;
    ncol = colIndex;
}

void EquilibriumFrictionSolver::computeGravityForce(Vector3d gravity, vector<double> &externalForce)
{
    externalForce.resize(graph->dynamic_nodes.size() * 6);
    for(int id = 0; id < graph->dynamic_nodes.size(); id++){
        externalForce[id * 6] = gravity(0);
        externalForce[id * 6 + 1] = gravity(1);
        externalForce[id * 6 + 2] = gravity(2);
    }
    return;
}