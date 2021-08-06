//
// Created by ziqwang on 2019-12-10.
//

#ifndef TOPOLITE_INTERLOCKINGSOLVER_H
#define TOPOLITE_INTERLOCKINGSOLVER_H
#include <Eigen/Sparse>
#include <Eigen/SparseCore>
#include <Eigen/Dense>
#include "TopoLite/Stability/ContactGraph/ContactGraphPlane.h"
#include "Utility/TopoObject.h"

using std::string;
using std::map;
using std::vector;
using std::shared_ptr;


template<typename Scalar>
class InterlockingSolver: public TopoObject{
public:
    typedef Matrix<double, 3, 1> Vector3;
    typedef Matrix<double, 1, 2> RowVector2;
    typedef Matrix<double, 1, 4> RowVector4;
    typedef Eigen::SparseMatrix<double, Eigen::ColMajor>  EigenSpMat;
    typedef Eigen::Triplet<double>  EigenTriple;
    typedef std::vector<Vector3> stdvec_Vector3;
    typedef shared_ptr<VPoint<Scalar>> pVertex;
    typedef weak_ptr<ContactGraphNode<Scalar>> wpContactGraphNode;

public:

    struct InterlockingData{
        double energy;
        stdvec_Vector3 translation;  // translation velocity
        stdvec_Vector3 rotation;    // rotation velocity
        stdvec_Vector3 center;      // rotational center
    };

public:

    shared_ptr<ContactGraphBase<Scalar>> graph;

public:

    InterlockingSolver(shared_ptr<ContactGraphBase<Scalar>> _graph): TopoObject(_graph->getVarList())
    {
        graph = _graph;
    }

public:

    /*************************************************
    *           Compute Interlocking Matrix
    *************************************************/


    void computeTranslationalInterlockingMatrix(vector<EigenTriple> &tri, Eigen::Vector2i &size);

    void computeRotationalInterlockingMatrix(vector<EigenTriple> &tri, Eigen::Vector2i &size);      // used to compute A

    void computeRotationalInterlockingMatrixDense(Eigen::MatrixXd &mat);                            // used to compute A

    void computeTranslationalInterlockingMatrixDense(Eigen::MatrixXd &mat);

    void computeRotationalInterlockingMatrixSparse(EigenSpMat &mat);

    void appendAuxiliaryVariables(vector<EigenTriple> &tri, Eigen::Vector2i &size);

    void appendMergeConstraints(vector<EigenTriple> &tri, Eigen::Vector2i &size, bool isRotation);

public:

    /*************************************************
    *           Interlocking Test
    *************************************************/

    virtual bool isTranslationalInterlocking(shared_ptr<InterlockingData> &data){ return  true;}

    virtual bool isRotationalInterlocking(shared_ptr<InterlockingData> &data){ return  true;}

};

#endif //TOPOLITE_INTERLOCKINGSOLVER_H
