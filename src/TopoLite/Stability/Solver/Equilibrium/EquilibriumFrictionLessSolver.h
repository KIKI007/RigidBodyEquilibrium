//
// Created by ziqwang on 03.08.20.
//

#ifndef TOPOLITE_EQUILIBRIUMFRICTIONLESSSOLVER_H
#define TOPOLITE_EQUILIBRIUMFRICTIONLESSSOLVER_H

#include "Utility/TopoObject.h"
#include "Stability/ContactGraph/ContactGraphBase.h"
#include <Eigen/Sparse>
#include <map>

using pairIJ = std::pair<int, int>;

template<int dim>
class EquilibriumFrictionLessSolver: public TopoObject {
public:
    typedef Matrix<double, 3, 1> Vector3;
    typedef Matrix<double, 1, 2> RowVector2;
    typedef Matrix<double, 1, 4> RowVector4;
    typedef Matrix<double, 1, 3> RowVector3;
    typedef Eigen::SparseMatrix<double, Eigen::ColMajor>  EigenSpMat;
    typedef Eigen::Triplet<double>  EigenTriple;
    typedef shared_ptr<VPoint<double>> pVertex;
    typedef weak_ptr<ContactGraphNode<double>> wpContactGraphNode;

public:
    struct StaticData{
        double energy;
        vector<Vector3> forces;
        vector<Vector3> points;
        vector<pairIJ> partIJs;
    };

    struct KinematicData{
        double energy;
        vector<Vector3> translation;  // translation velocity
        vector<Vector3> rotation;    // rotation velocity
        vector<Vector3> center;      // rotational center
    };

public:

    shared_ptr<ContactGraphBase<double>> graph;

    bool silence;

public:
    EquilibriumFrictionLessSolver(shared_ptr<ContactGraphBase<double>> _graph) : TopoObject(_graph->getVarList()){
        graph = _graph;
        silence = false;
    }

protected:

    /*************************************************
    *      Building block for the Interlocking Matrix
    *************************************************/

    void get_force_from_norm_fric(Vector3    n,    Vector3   u,  Vector3   v,
                                  RowVector2    &fkx, RowVector2 &fky, RowVector2 &fkz);

    void get_moment_from_norm_fric_vertex(Vector3 n,   Vector3 u,   Vector3 v, Vector3 r,
                                          RowVector2 &mx, RowVector2 &my, RowVector2 &mz);

    void get_A_j_k(int partID, int edgeID, Eigen::MatrixXd &Ajk, bool negativeForce = true, bool origin_as_centroid = true);

public:

    void computeEquilibriumMatrix(Eigen::MatrixXd &mat, bool negativeForce = true, bool origin_as_centroid = true);

    double computeEquilibriumMatrixConditonalNumber();

    virtual bool isEquilibrium(Vector3 gravity, shared_ptr<StaticData> &data){ return  true;}

    void getEquilibriumData(shared_ptr<StaticData> &data);

    void getKinetamticData(shared_ptr<KinematicData> &data, const vector<double> &solution);

    void getForces(Vector3d gravity, Eigen::VectorXd &force, bool origin_as_centroid = true);
};


#endif //TOPOLITE_EQUILIBRIUMFRICTIONLESSSOLVER_H
