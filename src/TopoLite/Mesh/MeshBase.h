//
// Created by ziqwang on 14.06.20.
//

#ifndef TOPOLITE_MESHBASE_H
#define TOPOLITE_MESHBASE_H
#include "Utility/TopoObject.h"
#include "Utility/GeometricPrimitives.h"
#include <igl/per_vertex_normals.h>
#include <igl/remove_duplicate_vertices.h>
template <typename Scalar>
class MeshBase : public TopoObject{
public:

    typedef Matrix<Scalar, 3, 1> Vector3;

    typedef Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;

    typedef Matrix<int, Eigen::Dynamic, Eigen::Dynamic> MatrixXi;

    using pTriangle = shared_ptr<Triangle<Scalar>> ;

public:

    MeshBase(shared_ptr<InputVarList> varList): TopoObject(varList){

    }

public:

    virtual void convertToTriLists(vector<pTriangle> &triList, bool compute_normal = false, bool convex = true) const{

    }

    virtual Vector3 center() const{
        return Vector3(0, 0, 0);
    }

    virtual size_t n_vertices() const {
        return 0;
    }

    virtual vector<Line<Scalar>> getWireFrame() const{
        return vector<Line<Scalar>>();
    };

public:


    void recomputeTriListsNormal(vector<pTriangle> &triList) const {
        MatrixX V(triList.size() * 3, 3);
        MatrixXi F(triList.size(), 3);

        int vID = 0;
        for(auto tri: triList){
            V.row(vID) = tri->v[0];
            V.row(vID + 1) = tri->v[1];
            V.row(vID + 2) = tri->v[2];
            F.row(vID / 3) = Eigen::Vector3i(vID, vID + 1, vID + 2);
            vID += 3;
        }

        MatrixX SV, N;
        MatrixXi SVI, SVJ, SF;
        igl::remove_duplicate_vertices(V, F, FLOAT_ERROR_LARGE, SV, SVI, SVJ, SF);
        igl::per_vertex_normals(SV, SF, igl::PerVertexNormalsWeightingType::PER_VERTEX_NORMALS_WEIGHTING_TYPE_DEFAULT, N);

        for(int id = 0; id < triList.size(); id++){
            for(int kd = 0; kd < 3; kd++){
                int vID = 3 * id + kd;
                int svID = SVJ(vID);
                triList[id]->vf[kd] = N.row(svID);
            }
        }

        return;
    }
};

#endif //TOPOLITE_MESHBASE_H
