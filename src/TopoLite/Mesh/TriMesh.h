//
// Created by ziqwang on 14.06.20.
//

#ifndef TOPOLITE_TRIMESH_H
#define TOPOLITE_TRIMESH_H
#include "MeshBase.h"
template <typename Scalar>
class TriMesh : public MeshBase<Scalar>{

public:

    typedef Matrix<Scalar, 3, 1> Vector3;

    using pTriangle = shared_ptr<Triangle<Scalar>> ;

public:

    vector<pTriangle> triList;

public:

    TriMesh(const vector<pTriangle> &_triList, shared_ptr<InputVarList> varList)
    :MeshBase<Scalar>(varList), triList(_triList)
    {

    }

public:

    void recomputeNormal(){
        MeshBase<Scalar>::recomputeTriListsNormal(triList);
    }

    void convertToTriLists(vector<pTriangle> &outputList, bool compute_normal = false, bool convex = true) const override
    {
        outputList = triList;
        if(compute_normal)
        {
            MeshBase<Scalar>::recomputeTriListsNormal(outputList);
        }
        return;
    }

    Vector3 center() const override{
        Vector3 center_pt(0, 0, 0);
        for(int id = 0; id < triList.size(); id++){
            center_pt += triList[id]->v[0] + triList[id]->v[1] + triList[id]->v[2];
        }
        center_pt /= triList.size() * 3;
        return center_pt;
    }

    size_t n_vertices() const override {
        return triList.size() * 3;
    }

    vector<Line<Scalar>> getWireFrame() const override{
        vector<Line<Scalar>> lines;
        for(int id = 0; id < triList.size(); id++)
        {
            if(triList[id])
            {
                Eigen::Vector3d p0, p1, p2;
                p0 = triList[id]->v[0];
                p1 = triList[id]->v[1];
                p2 = triList[id]->v[2];
                lines.push_back(Line<double>(p0, p1));
                lines.push_back(Line<double>(p1, p2));
                lines.push_back(Line<double>(p2, p0));
            }
        }
        return lines;
    }

};

#endif //TOPOLITE_TRIMESH_H
