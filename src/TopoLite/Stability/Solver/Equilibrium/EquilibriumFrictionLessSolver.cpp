//
// Created by ziqwang on 03.08.20.
//

#include "EquilibriumFrictionLessSolver.h"


/*************************************************
*
*      Building block for the Interlocking Matrix
*
*************************************************/

template<int dim>
void EquilibriumFrictionLessSolver<dim>::get_force_from_norm_fric(Vector3 n, Vector3 u, Vector3 v,
                                                                     RowVector2 &fkx, RowVector2 &fky, RowVector2 &fkz)
{
    Vector3 x_comp(1, 0 ,0);
    Vector3 y_comp(0, 1 ,0);
    Vector3 z_comp(0, 0 ,1);
    fkx = RowVector2(n.dot(x_comp), -n.dot(x_comp));
    fky = RowVector2(n.dot(y_comp), -n.dot(y_comp));
    fkz = RowVector2(n.dot(z_comp), -n.dot(z_comp));
    return;
}

template<int dim>
void EquilibriumFrictionLessSolver<dim>::get_moment_from_norm_fric_vertex(Vector3 n, Vector3 u, Vector3 v,
                                                                         Vector3 r, RowVector2 &mx, RowVector2 &my,
                                                                         RowVector2 &mz)
{
    Vector3 x_comp(1, 0 ,0);
    Vector3 y_comp(0, 1 ,0);
    Vector3 z_comp(0, 0 ,1);
    Vector3 m_normal = r.cross(n);
    Vector3 m_u_fric = r.cross(u);
    Vector3 m_v_fric = r.cross(v);

    mx = RowVector2(m_normal.dot(x_comp), -m_normal.dot(x_comp));
    my = RowVector2(m_normal.dot(y_comp), -m_normal.dot(y_comp));
    mz = RowVector2(m_normal.dot(z_comp), -m_normal.dot(z_comp));

    return;
}



template<int dim>
void EquilibriumFrictionLessSolver<dim>::get_A_j_k(int partID,
                                                   int edgeID,
                                                   Eigen::MatrixXd &Ajk,
                                                   bool negativeForce,
                                                   bool origin_as_centroid)
{
        // 1.0 get interface and part

        shared_ptr<ContactGraphEdgeBase<double>> edge = graph->edges[edgeID];
        shared_ptr<ContactGraphNode<double>> part = graph->nodes[partID];

        // 2.0 initialize the matrix Ajk

        int num_vks = edge->size();
        Ajk = Eigen::MatrixXd(6, (negativeForce ? 2 * num_vks : num_vks));

        for(size_t id = 0; id < edge->points.size(); id++)
        {
            // 3.0 filling the force fx, fy, fz, term

            Matrix<double, 3, 1> normal, u_fric, v_fric;
            RowVector2 fkx, fky, fkz;
            edge->get_norm_fric_for_block(partID, id, normal, u_fric, v_fric);
            get_force_from_norm_fric(normal.template cast<double>(), u_fric.template cast<double>(), v_fric.template cast<double>(), fkx, fky, fkz);

            if(negativeForce){
                Ajk.block(0, 2 * id, 1, 2) = fkx;
                Ajk.block(1, 2 * id, 1, 2) = fky;
                Ajk.block(2, 2 * id, 1, 2) = fkz;
            }
            else{
                Ajk(0, id) = fkx(0);
                Ajk(1, id) = fky(0);
                Ajk(2, id) = fkz(0);
            }

            // 4.0 filling the torque mx, my, mz term
            Vector3 r;
            if(origin_as_centroid){
                r = (edge->points[id]).template cast<double>();
            }
            else{
                r = (edge->points[id] - part->centerofmass).template cast<double>();
            }

            RowVector2 mx, my, mz;
            get_moment_from_norm_fric_vertex(normal.template cast<double>(), u_fric.template cast<double>(), v_fric.template cast<double>(), r, mx, my, mz);

            if(negativeForce) {
                Ajk.block(3, 2 * id, 1, 2) = mx;
                Ajk.block(4, 2 * id, 1, 2) = my;
                Ajk.block(5, 2 * id, 1, 2) = mz;
            }
            else{
                Ajk(3, id) = mx(0);
                Ajk(4, id) = my(0);
                Ajk(5, id) = mz(0);
            }
        }
    return;
}


template<int dim>
void EquilibriumFrictionLessSolver<dim>::computeEquilibriumMatrix(Eigen::MatrixXd &Aeq, bool negativeForce, bool origin_as_centroid) {
    // 1.0 init the matrix

    vector<int> row_start_index;
    int start_index = 0;

    for (shared_ptr<ContactGraphEdgeBase<double>> edge: graph->edges) {
        row_start_index.push_back(start_index);
//        std::cout << edge->partIDA << ",\t" << edge->partIDB << ",\t" << edge->size() << std::endl;
        start_index += edge->size() * (negativeForce? 2 : 1);
    }

    if(dim == 3){
        Aeq = Eigen::MatrixXd::Zero(graph->dynamic_nodes.size() * 6, start_index);

    }
    else if(dim == 2){
        Aeq = Eigen::MatrixXd::Zero(graph->dynamic_nodes.size() * 3, start_index);
    }
    else{
        return;
    }

    // 2.0 build matrix

    for (size_t id = 0; id < graph->edges.size(); id++)
    {
        shared_ptr<ContactGraphEdgeBase<double>> edge = graph->edges[id];
        shared_ptr<ContactGraphNode<double>> partA = graph->nodes[edge->partIDA];
        shared_ptr<ContactGraphNode<double>> partB = graph->nodes[edge->partIDB];
        int dyn_partIDA = partA->dynamicID;
        int dyn_partIDB = partB->dynamicID;

        int num_vk = edge->size();
        int num_col = negativeForce ? 2 * num_vk : num_vk;
        if (!partA->isBoundary)
        {
            Eigen::MatrixXd Ajk;
            get_A_j_k(partA->staticID, id, Ajk, negativeForce, origin_as_centroid);
            if(dim == 3){
                Aeq.block(6 * dyn_partIDA, row_start_index[id], 6, num_col) = Ajk;
            }
            else if(dim == 2){
                Aeq.block(3 * dyn_partIDA, row_start_index[id], 2, num_col) = Ajk.block(0, 0, 2, num_col);
                Aeq.block(3 * dyn_partIDA + 2, row_start_index[id], 1, num_col) = Ajk.block(5, 0, 1, num_col);
            }

        }
        if (!partB->isBoundary) {
            Eigen::MatrixXd Ajk;
            get_A_j_k(partB->staticID, id, Ajk, negativeForce, origin_as_centroid);

            if(dim == 3){
                Aeq.block(6 * dyn_partIDB, row_start_index[id], 6, num_col) = Ajk;
            }
            else if(dim == 2){
                Aeq.block(3 * dyn_partIDB, row_start_index[id], 2, num_col) = Ajk.block(0, 0, 2, num_col);
                Aeq.block(3 * dyn_partIDB + 2, row_start_index[id], 1, num_col) = Ajk.block(5, 0, 1, num_col);
            }
        }
    }
}

template <int dim>
double EquilibriumFrictionLessSolver<dim>::computeEquilibriumMatrixConditonalNumber()
{
    Eigen::MatrixXd Aeq;
    computeEquilibriumMatrix(Aeq);

    if(!Aeq.isZero()){
        Eigen::MatrixXd TAeq(Aeq.rows(), Aeq.cols() / 2);
        for(int id = 0; id < Aeq.cols() / 2; id++)
        {
            TAeq.col(id) = Aeq.col(id * 2);
        }
        Eigen::MatrixXd TransTAeq = TAeq.transpose();
        Eigen::MatrixXd TAeqinv = TAeq.completeOrthogonalDecomposition().pseudoInverse();
        return TAeqinv.norm() * TAeq.norm();
    }
    else{
        return 0;
    }
}

template <int dim>
void EquilibriumFrictionLessSolver<dim>::getEquilibriumData(shared_ptr<StaticData> &data){
    data = make_shared<StaticData>();
    for (size_t id = 0; id < graph->edges.size(); id++)
    {
        shared_ptr<ContactGraphEdgeBase<double>> edge = graph->edges[id];
        shared_ptr<ContactGraphNode<double>> partA = graph->nodes[edge->partIDA];
        shared_ptr<ContactGraphNode<double>> partB = graph->nodes[edge->partIDB];
        for(int jd = 0; jd < edge->points.size(); jd++)
        {
            Vector3 normal = edge->getContactNormal(partA->staticID, jd);
            data->forces.push_back(normal);
            Vector3 point = edge->points[jd];
            data->points.push_back(point);
            data->partIJs.push_back({partA->staticID, partB->staticID});
        }
    }
}

template<int dim>
void EquilibriumFrictionLessSolver<dim>::getKinetamticData(shared_ptr<KinematicData> &data, const vector<double> &solution)
{
    data = make_shared<KinematicData>();
    for (shared_ptr<ContactGraphNode<double>> node: graph->nodes)
    {
        Vector3 trans(0, 0, 0);
        Vector3 rotate(0, 0, 0);
        Vector3 center = Vector3(0, 0, 0);
        if (node->dynamicID != -1) {
            if (dim == 3) {
                trans = Vector3(solution[node->dynamicID * 6],
                                solution[node->dynamicID * 6 + 1],
                                solution[node->dynamicID * 6 + 2]);
                rotate = Vector3(-solution[node->dynamicID * 6 + 3],
                                  -solution[node->dynamicID * 6 + 4],
                                  -solution[node->dynamicID * 6 + 5]);
            }
            else if(dim == 2){
                trans = Vector3(solution[node->dynamicID * 3],
                                solution[node->dynamicID * 3 + 1],
                                0);
                rotate = Vector3(0,
                                  0,
                                  -solution[node->dynamicID * 3 + 2]);
            }
        }

        data->translation.push_back(trans);
        data->rotation.push_back(rotate);
        data->center.push_back(center);
    }
}

template <int dim>
void EquilibriumFrictionLessSolver<dim>::getForces(Vector3d gravity, Eigen::VectorXd &force, bool origin_as_centroid){

    int n = 0;
    if(dim == 3){
        n = 6;
    }
    else{
        n = 3;
    }
    force = Eigen::VectorXd::Zero(graph->dynamic_nodes.size() * n);

    for(int id = 0; id < graph->dynamic_nodes.size(); id++){
        Eigen::Vector3d g = gravity * graph->dynamic_nodes[id].lock()->mass;
        //force
        for(int kd = 0; kd < dim; kd++){
            force(id * n + kd) = -g(kd);
        }

        //torque
        if(origin_as_centroid){
            Eigen::Vector3d r = graph->dynamic_nodes[id].lock()->centerofmass.template cast<double>();
            if(dim == 3){
                for(int kd = 0; kd < 3; kd++){
                    force(id * 6 + kd + 3) = -r.cross(g)[kd];
                }
            }
            else if(dim == 2) {
                force(id * n + 2) = -r.cross(g)[2];
            }
        }
    }
}

template class EquilibriumFrictionLessSolver<3>;
template class EquilibriumFrictionLessSolver<2>;

