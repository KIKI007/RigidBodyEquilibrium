//
// Created by ziqwang on 03.08.20.
//

#include "ContactGraphBase.h"

template<typename Scalar>
ContactGraphBase<Scalar>::ContactGraphBase(const shared_ptr<InputVarList> &varList)
        : TopoObject(varList){
}

/**
 * @brief Class destructor
 * @tparam Scalar
 */
template<typename Scalar>
ContactGraphBase<Scalar>::~ContactGraphBase() {
    nodes.clear();
    edges.clear();
    dynamic_nodes.clear();
}


/*************************************************
*
*                  Graph Operation
*
*************************************************/
/**
 * @brief
 * @tparam Scalar
 * @param _node
 */
template<typename Scalar>
void ContactGraphBase<Scalar>::addNode(pContactGraphNode _node) {
    _node->staticID = nodes.size();

    nodes.push_back(_node);
}

/**
 * @brief
 * @tparam Scalar
 * @param _nodeA
 * @param _nodeB
 * @param _edge
 */
template<typename Scalar>
void ContactGraphBase<Scalar>::addContact(pContactGraphNode _nodeA, pContactGraphNode _nodeB, pContactGraphEdge _edge) {

    if (_nodeA->isBoundary && _nodeB->isBoundary)
        return;

    _edge->partIDA = _nodeA->staticID;
    _edge->partIDB = _nodeB->staticID;

    edges.push_back(_edge);

    pair<wpContactGraphNode, wpContactGraphEdge> contactNeighbor;

    contactNeighbor.first = _nodeB;
    contactNeighbor.second = _edge;
    _nodeA->neighbors.push_back(contactNeighbor);

    contactNeighbor.first = _nodeA;
    contactNeighbor.second = _edge;
    _nodeB->neighbors.push_back(contactNeighbor);
}

/**
 * @brief merging nodeA and nodeB. they should move together.
 * @tparam Scalar
 * @param _nodeA,_nodeB
 */
template<typename Scalar>
void ContactGraphBase<Scalar>::mergeNode(pContactGraphNode _nodeA, pContactGraphNode _nodeB) {
    merged_nodes.push_back({_nodeA, _nodeB});
}

/**
 * @brief Assign nodeID to the graph. Must be called after updating the graph
 * @tparam Scalar
 */
template<typename Scalar>
void ContactGraphBase<Scalar>::finalize() {
    int dynamicID = 0;
    dynamic_nodes.clear();
    for (pContactGraphNode node : nodes) {
        if (!node->isBoundary) {
            node->dynamicID = dynamicID++;
            dynamic_nodes.push_back(node);
        } else {
            node->dynamicID = -1;
        }
    }
}


template<typename Scalar>
shared_ptr<ContactGraphBase<Scalar>> ContactGraphBase<Scalar>::computeSubgraph(vector<int> nodeIDs)
{
    vector<bool> visited;
    visited.resize(nodes.size(), false);
    for(int id = 0; id < nodeIDs.size(); id++){
        visited[nodeIDs[id]] = true;
    }

    shared_ptr<ContactGraphBase<Scalar>> graph = make_shared<ContactGraphBase<Scalar>>(getVarList());

    vector<int> NodeIDMap;
    NodeIDMap.resize(nodes.size(), 0);
    int newStaticID = 0;


    for(int id = 0; id < nodes.size(); id++)
    {
        if(visited[nodes[id]->staticID])
        {
            shared_ptr<ContactGraphNode<Scalar>> node = make_shared<ContactGraphNode<Scalar>>(*nodes[id]);
            int oldStaticID = node->staticID;
            node->staticID = newStaticID;
            NodeIDMap[oldStaticID] = newStaticID;
            newStaticID++;

            graph->nodes.push_back(node);
        }
    }

    for(int id = 0; id < edges.size(); id++)
    {
        int oldPartIDA = edges[id]->partIDA;
        int oldPartIDB = edges[id]->partIDB;

        if(visited[oldPartIDA] && visited[oldPartIDB]){
            shared_ptr<ContactGraphEdgeBase<Scalar>> edge = make_shared<ContactGraphEdgeBase<Scalar>>(*edges[id]);
            edge->partIDA = NodeIDMap[oldPartIDA];
            edge->partIDB = NodeIDMap[oldPartIDB];
            graph->edges.push_back(edge);
        }
    }

    graph->finalize();
    return graph;
}

/**
 * @brief return the all contact polygons as a mesh
 * @tparam Scalar
 * @param mesh
 */
template<typename Scalar>
void ContactGraphBase<Scalar>::getContactMesh(pPolyMesh &mesh) {
    mesh.reset();
    mesh = make_shared<PolyMesh<Scalar>>(getVarList());
    for (pContactGraphEdge edge: edges) {
        for (pPolygon poly: edge->polygons) {
            pPolygon copy_poly = make_shared<_Polygon<Scalar>>(*poly);
            if(edge->partIDA > edge->partIDB){
                copy_poly->reverseVertices();
            }
            mesh->polyList.push_back(copy_poly);
        }
    }

    if (mesh) mesh->removeDuplicatedVertices();

//    mesh->ScaleMesh(1.0 / normalized_scale);
//    mesh->TranslateMesh(-normalized_trans);
}


/**
 * @brief return the sampled contact points as vector<>
 * @tparam Scalar
 * @param mesh
 */

template<typename Scalar>
void ContactGraphBase<Scalar>::getContactPoints(vector<Vector3> &points){
    for (pContactGraphEdge edge: edges) {
        for (auto point: edge->points) {
            points.push_back(point);
        }
    }
}

template<typename Scalar>
void ContactGraphBase<Scalar>::getContactNormals(vector<Line<double>> &normal_lines,
                                                 double length){
    normal_lines.clear();
    for (pContactGraphEdge edge: edges)
    {
        for(int id = 0; id < edge->points.size(); id++){
            Vector3d p0 = edge->points[id].template cast<double>();
            Vector3d n = edge->normals[id].template cast<double>();
            Vector3d p1 = p0 + n.normalized() * length;
            normal_lines.push_back(Line<double>(p0, p1));
        }
    }
}

template<typename Scalar>
void ContactGraphBase<Scalar>::getContactNormals(vector<Line<double>> &normal_lines,
                                                 int partIDA,
                                                 int partIDB,
                                                 double length){
    normal_lines.clear();
    if(partIDA > partIDB){
        std::swap(partIDA, partIDB);
    }
    for (pContactGraphEdge edge: edges)
    {
        if(edge->partIDA == partIDA && edge->partIDB == partIDB)
        {
            for(int id = 0; id < edge->points.size(); id++)
            {
                Vector3d p0 = edge->points[id].template cast<double>();
                Vector3d n = edge->normals[id].template cast<double>();
                Vector3d p1 = p0 + n.normalized() * length;
                normal_lines.push_back(Line<double>(p0, p1));
            }
        }
    }
}

template class ContactGraphBase<double>;
template class ContactGraphBase<float>;