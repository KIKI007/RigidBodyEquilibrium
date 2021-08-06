//
// Created by ziqwang on 14.01.19.
//

#ifndef TOPOLOCKCREATOR_CONTACTGRAPH_H
#define TOPOLOCKCREATOR_CONTACTGRAPH_H

#include "ContactGraphEdgePlane.h"
#include "ContactGraphBase.h"

#include "Utility/PolyPolyBoolean.h"

#include <string>
#include <map>

#include <iostream>
#include <algorithm>
#include <set>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_sort.h>
#include <cmath>

using pairIJ = std::pair<int, int>;

template<typename Scalar>
class ContactGraphPlane : public ContactGraphBase<Scalar>{
public:
    typedef shared_ptr<ContactGraphNode<Scalar>> pContactGraphNode;

    typedef shared_ptr<ContactGraphEdgeBase<Scalar>> pContactGraphEdge;

    typedef weak_ptr<ContactGraphEdgeBase<Scalar>> wpContactGraphEdge;

    typedef weak_ptr<ContactGraphNode<Scalar>> wpContactGraphNode;

    typedef shared_ptr<_Polygon<Scalar>> pPolygon;

    typedef weak_ptr<_Polygon<Scalar>> wpPolygon;

    typedef Matrix<Scalar, 3, 1> Vector3;

    typedef shared_ptr<PolyMesh<Scalar>> pPolyMesh;

    struct polygonal_face{
        Matrix<Scalar, 3, 1> nrm;
        Matrix<Scalar, 3, 1> center;
        double D;
        int partID;
        int groupID;
        wpPolygon polygon;
        double eps;
    };

    struct plane_contact_compare
    {
    public:
        bool operator()(const polygonal_face &A, const polygonal_face &B) const
        {
            double eps = A.eps / 2;

            if (A.nrm[0] - B.nrm[0] < -eps)
                return true;
            if (A.nrm[0] - B.nrm[0] > eps)
                return false;

            if (A.nrm[1] - B.nrm[1] < -eps)
                return true;
            if (A.nrm[1] - B.nrm[1] > eps)
                return false;

            if (A.nrm[2] - B.nrm[2] < -eps)
                return true;
            if (A.nrm[2] - B.nrm[2] > eps)
                return false;

            if (A.D - B.D < -eps)
                return true;
            if (A.D - B.D > eps)
                return false;

            return false;
        }
    };

public:
    using ContactGraphBase<Scalar>::nodes;

    using ContactGraphBase<Scalar>::edges;

    using ContactGraphBase<Scalar>::merged_nodes;

private:

    // Class attributes used in constructFromPolyMesh method - Should not be accessed
    vector<pPolyMesh> meshes_input;

    vector<polygonal_face> contact_faces;

    vector<pairIJ> contact_pairs;

    vector<shared_ptr<ContactGraphEdgePlane<Scalar>>> contact_graphedges;

public:

    explicit ContactGraphPlane(const shared_ptr<InputVarList> &varList);

    ~ContactGraphPlane();

public:

    bool buildFromMeshes(const vector<pPolyMesh> &meshes,
                         vector<bool> &atBoundary,
                         Scalar dist_eps = 0.002,
                         Scalar angle_eps = 5.0 / 180.0 * M_PI,
                         bool convexhull = true);

    using ContactGraphBase<Scalar>::finalize;

    using ContactGraphBase<Scalar>::getVarList;

    using ContactGraphBase<Scalar>::addNode;

    using ContactGraphBase<Scalar>::addContact;

private:

    /*************************************************
    *
    *             constructFromPolyMesh methods
    *
    *************************************************/
    
    bool clusterFacesofInputMeshes(Scalar dist_eps, Scalar angle_eps);

    void listPotentialContacts(vector<bool> &atBoundary, double angle_eps);

    void computeContacts();

    void buildNodes(vector<bool> &atBoundary);

    void buildEdges();

    void computeConvexHullofEdgePolygons();
};

#endif //TOPOLOCKCREATOR_CONTACTGRAPH_H
