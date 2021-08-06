//
// Created by ziqwang on 22.10.20.
//

#ifndef TOPOLITE_INTERLOCKINGSOLVER_MOSEK_H
#define TOPOLITE_INTERLOCKINGSOLVER_MOSEK_H

#include "InterlockingSolver.h"
#include <Eigen/SparseQR>
#include "fusion.h"

template <typename Scalar>
class InterlockingSolver_Mosek : public InterlockingSolver<Scalar> {
public:
    typedef shared_ptr<typename InterlockingSolver<Scalar>::InterlockingData> pInterlockingData;
    typedef Eigen::SparseMatrix<double, Eigen::ColMajor> EigenSpMat;
    typedef Eigen::Triplet<double> EigenTriple;
    typedef shared_ptr<ContactGraphBase<Scalar>> pContactGraph;
    typedef shared_ptr<ContactGraphNode<Scalar>> pContactGraphNode;
    typedef Eigen::Matrix<double, 3, 1> Vector3;

    using InterlockingSolver<Scalar>::graph;

public:
    InterlockingSolver_Mosek(pContactGraph _graph) :
            InterlockingSolver<Scalar>::InterlockingSolver(_graph) {}

public:

    bool isTranslationalInterlocking(pInterlockingData &data);

    bool isRotationalInterlocking(pInterlockingData &data);

    bool checkSpecialCase(pInterlockingData &data,
                          vector<EigenTriple> copy_tris,
                          bool rotationalInterlockingCheck,
                          Eigen::Vector2i copy_size);

    bool solve(pInterlockingData &data,
               vector<EigenTriple> &tris,
               bool rotationalInterlockingCheck,
               int num_row,
               int num_col,
               int num_var);

    void unpackSolution(InterlockingSolver_Mosek::pInterlockingData &data,
                        bool rotationalInterlockingCheck,
                        const double *solution,
                        int num_var);

    void toMosekMatrix(int row, int col, vector<Eigen::Triplet<double>> &tripleList, mosek::fusion::Matrix::t &mat);
};

#endif //TOPOLITE_INTERLOCKINGSOLVER_MOSEK_H
