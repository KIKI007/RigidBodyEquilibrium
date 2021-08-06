//
// Created by ziqwang on 14.08.20.
//

#ifndef TOPOLITE_EQUILIBRIUMFRICTIONSOLVER_H
#define TOPOLITE_EQUILIBRIUMFRICTIONSOLVER_H

#include "Utility/TopoObject.h"
#include "Stability/ContactGraph/ContactGraphBase.h"
#include <Eigen/Sparse>
#include <map>

using pairIJ = std::pair<int, int>;

template <typename Scalar>
class EquilibriumFrictionSolver : public TopoObject {

public:
    typedef Matrix<double, 3, 1> Vector3;
    typedef Matrix<double, 1, 2> RowVector2;
    typedef Matrix<double, 1, 4> RowVector4;
    typedef Matrix<double, 1, 3> RowVector3;
    typedef Eigen::SparseMatrix<double, Eigen::ColMajor>  EigenSpMat;
    typedef Eigen::Triplet<double>  EigenTriple;
    typedef std::vector<Vector3,Eigen::aligned_allocator<Vector3>> stdvec_Vector3;
    typedef shared_ptr<VPoint<Scalar>> pVertex;
    typedef weak_ptr<ContactGraphNode<Scalar>> wpContactGraphNode;

public:
    struct EquilibriumData{
        double infeasibility;
        stdvec_Vector3 force;
        stdvec_Vector3 torque;
        stdvec_Vector3 contact_points;
        vector<pairIJ> partIJ;
    };

public:
    shared_ptr<ContactGraphBase<Scalar>> graph;

public:
    EquilibriumFrictionSolver(shared_ptr<ContactGraphBase<Scalar>> _graph) : TopoObject(_graph->getVarList()){
        graph = _graph;
    }

public:

    void get_moment_from_norm_fric_vertex(Vector3 n,   Vector3 u,   Vector3 v, Vector3 r,
                                          RowVector4 &mx, RowVector4 &my, RowVector4 &mz);

    void get_force_from_norm_fric(Vector3    n,    Vector3   u,  Vector3   v,
                                  RowVector4    &fkx, RowVector4 &fky, RowVector4 &fkz);

    void get_A_j_k(int partID, int edgeID, Eigen::MatrixXd &Ajk, bool negativeForce = true);

public:

    void computeEquilibriumMatrix(Eigen::MatrixXd &mat, bool negativeForce = true);

    virtual bool isEquilibrium(Vector3 gravity, shared_ptr<EquilibriumData> &data){ return  true;}

};


#endif //TOPOLITE_EQUILIBRIUMFRICTIONSOLVER_H
