//
// Created by ziqwang on 09.08.21.
//

#ifndef RIGIDBODYEQUILIBRIUM_EQUILIBRIUMFRICTIONSOLVER_MOSEK_H
#define RIGIDBODYEQUILIBRIUM_EQUILIBRIUMFRICTIONSOLVER_MOSEK_H

#include "EquilibriumFrictionSolver.h"
#include "mosek.h"
#include "fusion.h"

class EquilibriumFrictionSolver_Mosek : public EquilibriumFrictionSolver{

public:
    EquilibriumFrictionSolver_Mosek(pContactGraph _graph) :
    EquilibriumFrictionSolver_Mosek::EquilibriumFrictionSolver(_graph) {}

public:

    bool isEquilibrium(Vector3 gravity, shared_ptr<StaticData> &data) override;

    void unpackStaticData(const vector<double> &forces_norm, shared_ptr<StaticData> &data);

private:

    void toMosekMatrix(int row,
                       int col,
                       vector<Eigen::Triplet<double>> &tripleList,
                       mosek::fusion::Matrix::t &mat){

        int n_value = tripleList.size();
        std::shared_ptr<monty::ndarray<int, 1>> row_index (new monty::ndarray<int,1>(monty::shape(n_value), [tripleList](ptrdiff_t i) {return tripleList[i].row();}));
        std::shared_ptr<monty::ndarray<int, 1>> col_index (new monty::ndarray<int,1>(monty::shape(n_value), [tripleList](ptrdiff_t i) {return tripleList[i].col();}));
        std::shared_ptr<monty::ndarray<double, 1>> m_content (new monty::ndarray<double,1>(monty::shape(n_value), [tripleList](ptrdiff_t i) {return tripleList[i].value();}));
        mat = mosek::fusion::Matrix::sparse(row, col, row_index, col_index, m_content);
    }
};


#endif //RIGIDBODYEQUILIBRIUM_EQUILIBRIUMFRICTIONSOLVER_MOSEK_H
