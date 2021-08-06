//
// Created by robinjodon on 23.10.20.
//
#include "InterlockingSolver_Mosek.h"

/* ----------------------------------------------------------------------------------------------------------------- */
/* ----IPOPT INTERLOCKING SOLVER------------------------------------------------------------------------------------ */
/* ----------------------------------------------------------------------------------------------------------------- */

template<typename Scalar>
bool InterlockingSolver_Mosek<Scalar>::isTranslationalInterlocking(InterlockingSolver_Mosek::pInterlockingData &data) {
    vector<EigenTriple> tris;
    Eigen::Vector2i size;

    InterlockingSolver<Scalar>::computeTranslationalInterlockingMatrix(tris, size);

    if (!checkSpecialCase(data, tris, false, size)) {
        return false;
    }

    int num_var = size[1];
    InterlockingSolver<Scalar>::appendAuxiliaryVariables(tris, size);
    InterlockingSolver<Scalar>::appendMergeConstraints(tris, size, false);

    return solve(data, tris, false, size[0], size[1], num_var);
}

template<typename Scalar>
bool InterlockingSolver_Mosek<Scalar>::isRotationalInterlocking(InterlockingSolver_Mosek::pInterlockingData &data) {
    vector<EigenTriple> tris;
    Eigen::Vector2i size;

    InterlockingSolver<Scalar>::computeRotationalInterlockingMatrix(tris, size);

//    std::cout << "special case" << std::endl;
    if (!checkSpecialCase(data, tris, true, size)) {
        return false;
    }

    int num_var = size[1];
    InterlockingSolver<Scalar>::appendAuxiliaryVariables(tris, size);
    InterlockingSolver<Scalar>::appendMergeConstraints(tris, size, true);

//    std::cout << "solve" << std::endl;
    return solve(data, tris, true, size[0], size[1], num_var);
}

template <typename Scalar>
bool InterlockingSolver_Mosek<Scalar>::checkSpecialCase(pInterlockingData &data,
                                                        vector<EigenTriple> copy_tris,
                                                        bool rotationalInterlockingCheck,
                                                        Eigen::Vector2i copy_size) {
    InterlockingSolver<Scalar>::appendMergeConstraints(copy_tris, copy_size, rotationalInterlockingCheck);

    if (copy_size[0] < copy_size[1]) return true;

    EigenSpMat A(copy_size[0], copy_size[1]);
    A.setFromTriplets(copy_tris.begin(), copy_tris.end());

    Eigen::SparseQR<Eigen::SparseMatrix<double>,
            Eigen::COLAMDOrdering<int> > solver;
    solver.compute(A.transpose());

    if (A.cols() - solver.rank() == 0) {
        return true;
    } else {
        Eigen::VectorXd solution = Eigen::MatrixXd(solver.matrixQ()).rightCols(A.cols() - solver.rank()).col(0);
        std::cout << "||A * x||: " << (A * solution).norm() << ", ||x||: " << solution.norm() << std::endl;
        unpackSolution(data, rotationalInterlockingCheck, solution.data(), copy_size[1]);
        return false;
    }
    return true;
}

template<typename Scalar>
void InterlockingSolver_Mosek<Scalar>::toMosekMatrix(int row, int col, vector<Eigen::Triplet<double>> &tripleList, mosek::fusion::Matrix::t &mat){
    int n_value = tripleList.size();
    shared_ptr<monty::ndarray<int, 1>> row_index (new monty::ndarray<int,1>(monty::shape(n_value), [tripleList](ptrdiff_t i) {return tripleList[i].row();}));
    shared_ptr<monty::ndarray<int, 1>> col_index (new monty::ndarray<int,1>(monty::shape(n_value), [tripleList](ptrdiff_t i) {return tripleList[i].col();}));
    shared_ptr<monty::ndarray<double, 1>> m_content (new monty::ndarray<double,1>(monty::shape(n_value), [tripleList](ptrdiff_t i) {return tripleList[i].value();}));
    mat = mosek::fusion::Matrix::sparse(row, col, row_index, col_index, m_content);
}

template<typename Scalar>
bool InterlockingSolver_Mosek<Scalar>::solve(InterlockingSolver_Mosek::pInterlockingData &data, vector<EigenTriple> &tris,
                                             bool rotationalInterlockingCheck,
                                             int num_row,
                                             int num_col,
                                             int num_var) {

// Create a model with the name 'lo1'
    mosek::fusion::Model::t model = new mosek::fusion::Model("interlocking");

    auto _model = monty::finally([&]() { model->dispose(); });

    //collison matrix
    vector<EigenTriple> col_mat_triplet;
    mosek::fusion::Matrix::t mosek_A;
    toMosekMatrix(num_row, num_col, tris, mosek_A);
    EigenSpMat A (num_row, num_col);
    A.setFromTriplets(col_mat_triplet.begin(), col_mat_triplet.end());

    // Create variable 'v' of length n_col
    mosek::fusion::Variable::t var_v = model->variable("v", num_var);

    // Create variable 't' of length n_row

    mosek::fusion::Variable::t var_t = model->variable("t", num_col - num_var);

    // Create constraints
    auto con0 = model->constraint(mosek::fusion::Expr::mul(mosek_A, mosek::fusion::Expr::vstack(var_v, var_t)), mosek::fusion::Domain::greaterThan(0.0));

    auto con1 = model->constraint(var_t, mosek::fusion::Domain::greaterThan(0.0));
    auto con2 = model->constraint(var_v, mosek::fusion::Domain::inRange(-1.0, 1.0));

    // Set the objective function to (g^v + 1/2v*v)
    model->objective("infeasibility", mosek::fusion::ObjectiveSense::Maximize, mosek::fusion::Expr::sum(var_t));

    // Solve the problem
    model->solve();

    double target_obj_value = model->primalObjValue();

    shared_ptr<monty::ndarray<double, 1>> mosek_v = var_v->level();
    shared_ptr<monty::ndarray<double, 1>> mosek_t = var_t->level();
    shared_ptr<monty::ndarray<double, 1>> mosek_con = con0->level();

    vector<double> solution;
    for(int id = 0; id < mosek_v->size(); id++){
        solution.push_back((*mosek_v)[id]);
    }

    unpackSolution(data, rotationalInterlockingCheck, solution.data(), num_var);

    double min_row_sol = MAX_FLOAT;
    for (int id = 0; id < num_row; id++) {
        min_row_sol = std::min((*mosek_con)[id], min_row_sol);
    }

    double max_t = 0;
    for (int id = 0; id < mosek_t->size(); id++) {
        max_t = std::max((*mosek_t)[id], max_t);
    }

    std::cout << "min_row:\t" << min_row_sol; //should be around zero
    std::cout << ",\tmax_t:\t" << std::abs(max_t); //interlocking if max_t is around zero
    std::cout << ",\taverage_t:\t" << std::abs(target_obj_value) / num_row << std::endl;//interlocking if average_t is around zero

    data->energy = model->primalObjValue();

    if (max_t < 5e-6) {
        return true;
    } else {
        return false;
    }
}

template<typename Scalar>
void InterlockingSolver_Mosek<Scalar>::unpackSolution(InterlockingSolver_Mosek::pInterlockingData &data,
                                                      bool rotationalInterlockingCheck,
                                                      const double *solution,
                                                      int num_var) {
    data = make_shared<typename InterlockingSolver<Scalar>::InterlockingData>();
    for (pContactGraphNode node: graph->nodes) {
        Vector3 trans(0, 0, 0);
        Vector3 rotate(0, 0, 0);
        Vector3 center = (node->centroid).template cast<double>();
        if (node->dynamicID != -1) {
            if (rotationalInterlockingCheck) {
                trans = Vector3(solution[node->dynamicID * 6],
                                solution[node->dynamicID * 6 + 1],
                                solution[node->dynamicID * 6 + 2]);
                rotate = -Vector3(solution[node->dynamicID * 6 + 3],
                                  solution[node->dynamicID * 6 + 4],
                                  solution[node->dynamicID * 6 + 5]);
            } else {
                trans = Vector3(solution[node->dynamicID * 3],
                                solution[node->dynamicID * 3 + 1],
                                solution[node->dynamicID * 3 + 2]);
            }
        }
        data->translation.push_back(trans);
        data->rotation.push_back(rotate);
        data->center.push_back(center);

//        std::cout << node->staticID << ":" << trans.transpose() << ", " << rotate.transpose() << std::endl;
    }
}

template class InterlockingSolver_Mosek<float>;
template class InterlockingSolver_Mosek<double>;