//
// Created by ziqwang on 14.08.20.
//

#include "EquilibriumFrictionSolver.h"



template<typename Scalar>
void EquilibriumFrictionSolver<Scalar>::get_force_from_norm_fric(Vector3    n, Vector3   u, Vector3   v,
                                                                     RowVector4    &fkx, RowVector4 &fky, RowVector4 &fkz){
    Vector3 x_comp(1, 0 ,0);
    Vector3 y_comp(0, 1 ,0);
    Vector3 z_comp(0, 0 ,1);
    fkx = RowVector4(n.dot(x_comp), -n.dot(x_comp), u.dot(x_comp), v.dot(x_comp));
    fky = RowVector4(n.dot(y_comp), -n.dot(y_comp), u.dot(y_comp), v.dot(y_comp));
    fkz = RowVector4(n.dot(z_comp), -n.dot(z_comp), u.dot(z_comp), v.dot(z_comp));
}

template<typename Scalar>
void EquilibriumFrictionSolver<Scalar>::get_moment_from_norm_fric_vertex(Vector3 n, Vector3 u, Vector3 v, Vector3 r,
                                                                             RowVector4 &mx, RowVector4 &my, RowVector4 &mz)
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

template <typename Scalar>
void EquilibriumFrictionSolver<Scalar>::get_A_j_k(int partID, int edgeID, Eigen::MatrixXd &Ajk, bool negativeForce) {

    // 1.0 get interface and part

    shared_ptr<ContactGraphEdgeBase<Scalar>> edge = graph->edges[edgeID];
    shared_ptr<ContactGraphNode<Scalar>> part = graph->nodes[partID];

    // 2.0 initialize the matrix Ajk

    int num_vks = edge->size();
    Ajk = Eigen::MatrixXd(6, (negativeForce ? 4 * num_vks : 3 * num_vks));
    for(size_t id = 0; id < edge->polygons.size(); id++)
    {
        // 3.0 filling the force fx, fy, fz, term

        Matrix<Scalar, 3, 1> normal, u_fric, v_fric;
        RowVector4 fkx, fky, fkz;
        edge->get_norm_fric_for_block(partID, id, normal, u_fric, v_fric);
        get_force_from_norm_fric(normal.template cast<double>(), u_fric.template cast<double>(), v_fric.template cast<double>(), fkx, fky, fkz);

        if(negativeForce){
            Ajk.block(0, 4 * id, 1, 4 ) = fkx;
            Ajk.block(1, 4 * id, 1, 4 ) = fky;
            Ajk.block(2, 4 * id, 1, 4 ) = fkz;
        }
        else{
            RowVector3 nfkx = RowVector3(fkx(0), fkx(2), fkx(3));
            RowVector3 nfky = RowVector3(fky(0), fky(2), fky(3));
            RowVector3 nfkz = RowVector3(fkz(0), fkz(2), fkz(3));

            Ajk.block(0, 3 * id, 1, 3 ) = nfkx;
            Ajk.block(1, 3 * id, 1, 3 ) = nfky;
            Ajk.block(2, 3 * id, 1, 3 ) = nfkz;
        }


        // 4.0 filling the torque mx, my, mz term
        Vector3 r = (edge->points[id] - part->centroid).template cast<double>();
        RowVector4 mx, my, mz;
        get_moment_from_norm_fric_vertex(normal.template cast<double>(), u_fric.template cast<double>(), v_fric.template cast<double>(), r, mx, my, mz);

        if(negativeForce){
            Ajk.block(3, 4 * id, 1, 4) = mx;
            Ajk.block(4, 4 * id, 1, 4) = my;
            Ajk.block(5, 4 * id, 1, 4) = mz;
        }
        else{
            RowVector3 nmx = RowVector3(mx(0), mx(2), mx(3));
            RowVector3 nmy = RowVector3( my(0), my(2), my(3));
            RowVector3 nmz = RowVector3(mz(0), mz(2), mz(3));

            Ajk.block(3, 3 * id, 1, 3 ) = nmx;
            Ajk.block(4, 3 * id, 1, 3 ) = nmy;
            Ajk.block(5, 3 * id, 1, 3 ) = nmz;
        }
    }
}

template <typename Scalar>
void EquilibriumFrictionSolver<Scalar>::computeEquilibriumMatrix(Eigen::MatrixXd &Aeq, bool negativeForce) {

    // 1.0 init the matrix

    vector<int> row_start_index;
    int start_index = 0;
    for (shared_ptr<ContactGraphEdgeBase<Scalar>> edge: graph->edges) {
        row_start_index.push_back(start_index);
        // std::cout << edge->partIDA << ",\t" << edge->partIDB << ",\t" << edge->num_points() << std::endl;
        start_index += edge->size() * (negativeForce ? 4: 3);
    }
    Aeq = Eigen::MatrixXd::Zero(graph->dynamic_nodes.size() * 6, start_index);

    // 2.0 build matrix

    for (size_t id = 0; id < graph->edges.size(); id++) {
        shared_ptr<ContactGraphEdgeBase<Scalar>> edge = graph->edges[id];
        shared_ptr<ContactGraphNode<Scalar>> partA = graph->nodes[edge->partIDA];
        shared_ptr<ContactGraphNode<Scalar>> partB = graph->nodes[edge->partIDB];
        int dyn_partIDA = partA->dynamicID;
        int dyn_partIDB = partB->dynamicID;
        int num_vk = edge->size();
        if (!partA->isBoundary) {
            Eigen::MatrixXd Ajk;
            get_A_j_k(partA->staticID, id, Ajk, negativeForce);
            Aeq.block(6 * dyn_partIDA, row_start_index[id], 6, negativeForce ? 4 * num_vk : 3 * num_vk) = Ajk;
        }
        if (!partB->isBoundary) {
            Eigen::MatrixXd Ajk;
            get_A_j_k(partB->staticID, id, Ajk, negativeForce);
            Aeq.block(6 * dyn_partIDB, row_start_index[id], 6, negativeForce ? 4 * num_vk : 3 * num_vk) = Ajk;
        }
    }
}