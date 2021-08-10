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

class EquilibriumFrictionSolver : public TopoObject {
public:
    typedef Matrix<double, 3, 1> Vector3;
    typedef Matrix<double, 1, 2> RowVector2;
    typedef Matrix<double, 1, 4> RowVector4;
    typedef Matrix<double, 1, 3> RowVector3;
    typedef Eigen::SparseMatrix<double, Eigen::ColMajor>  EigenSpMat;
    typedef Eigen::Triplet<double>  EigenTriple;
    typedef std::vector<Vector3,Eigen::aligned_allocator<Vector3>> stdvec_Vector3;
    typedef shared_ptr<VPoint<double>> pVertex;
    typedef weak_ptr<ContactGraphNode<double>> wpContactGraphNode;
    typedef shared_ptr<ContactGraphBase<double>> pContactGraph;

public:

    struct StaticData{
        double energy;
        vector<Vector3> compression_forces;
        vector<Vector3> tension_forces;
        vector<Vector3> friction_forces;
        vector<Vector3> contact_points;
        vector<pairIJ> partIJs;
    };

public:
    shared_ptr<ContactGraphBase<double>> graph;

public:
    EquilibriumFrictionSolver(shared_ptr<ContactGraphBase<double>> _graph) : TopoObject(_graph->getVarList()){
        graph = _graph;
    }

public:

    void get_moment_from_norm_fric_vertex(Vector3 n, Vector3 u, Vector3 v, Vector3 r,
                                          RowVector4 &mx, RowVector4 &my, RowVector4 &mz);

    void get_force_from_norm_fric(Vector3 n, Vector3 u, Vector3 v,
                                  RowVector4 &fkx, RowVector4 &fky, RowVector4 &fkz);

    void get_A_j_k(int partID, int edgeID, Eigen::MatrixXd &Ajk);

public:

    void computeEquilibriumMatrix(vector<EigenTriple> &tripleList, int &nrow, int &ncol);

    void computeFrictionMatrix(vector<EigenTriple> &tripleList, int &now, int &ncol);

    void computeGravityForce(Vector3d gravity, vector<double> &externalForce);

    virtual bool isEquilibrium(Vector3 gravity, shared_ptr<StaticData> &data){ return  true;}



private:

    void appendToTripleList(int staRowIndex,
                            int staColIndex,
                            const Eigen::MatrixXd &mat,
                            vector<EigenTriple> &tripleList){
        for(int ir = 0; ir < mat.rows(); ir++){
            for(int ic = 0; ic < mat.cols(); ic++){
                EigenTriple triple(staRowIndex + ir, staColIndex + ic, mat(ir, ic));
                tripleList.push_back(triple);
            }
        }
    }
};


#endif //TOPOLITE_EQUILIBRIUMFRICTIONSOLVER_H
