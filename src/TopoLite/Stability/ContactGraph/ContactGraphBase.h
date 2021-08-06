//
// Created by ziqwang on 03.08.20.
//

#ifndef TOPOLITE_CONTACTGRAPHBASE_H
#define TOPOLITE_CONTACTGRAPHBASE_H

#include "ContactGraphNode.h"

#include "ContactGraphEdgeBase.h"

#include "Mesh/PolyMesh.h"

template <typename Scalar>
class ContactGraphBase : public TopoObject{
public:
    typedef shared_ptr<ContactGraphNode<Scalar>> pContactGraphNode;

    typedef shared_ptr<ContactGraphEdgeBase<Scalar>> pContactGraphEdge;

    typedef weak_ptr<ContactGraphEdgeBase<Scalar>> wpContactGraphEdge;

    typedef weak_ptr<ContactGraphNode<Scalar>> wpContactGraphNode;

    typedef shared_ptr<_Polygon<Scalar>> pPolygon;

    typedef weak_ptr<_Polygon<Scalar>> wpPolygon;

    typedef Matrix<Scalar, 3, 1> Vector3;

    typedef shared_ptr<PolyMesh<Scalar>> pPolyMesh;

public:

    vector<pContactGraphNode> nodes;

    vector<pContactGraphEdge> edges;

    vector<std::pair<wpContactGraphNode, wpContactGraphNode>> merged_nodes;
public:

    //automatic generate
    vector<wpContactGraphNode> dynamic_nodes;

public:

    explicit ContactGraphBase(const shared_ptr<InputVarList> &varList);

    ~ContactGraphBase();

public:

    /*************************************************
    *
    *                  Graph Operation
    *
    *************************************************/

    void addNode(pContactGraphNode _node);

    void addContact(pContactGraphNode _nodeA, pContactGraphNode _nodeB, pContactGraphEdge _edge);

    void mergeNode(pContactGraphNode _nodeA, pContactGraphNode _nodeB);

    void finalize();

    void getContactMesh(pPolyMesh &mesh);

    void getContactPoints(vector<Vector3> &points);

    void getContactNormals(vector<Line<double>> &normal_lines, double length);

    void getContactNormals(vector<Line<double>> &normal_lines, int partIDA, int partIDB, double length);

    shared_ptr<ContactGraphBase<Scalar>> computeSubgraph(vector<int> nodeIDs);
};


#endif //TOPOLITE_CONTACTGRAPHBASE_H
