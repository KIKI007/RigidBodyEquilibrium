//
// Created by ziqwang on 14.01.19.
//


#include "ContactGraphPlane.h"
#include "Utility/ConvexHull2D.h"

/*************************************************
*
*                  Basic Operation
*
*************************************************/

/**
 * @brief Class constructor
 * @tparam Scalar
 * @param varList
 */
template<typename Scalar>
ContactGraphPlane<Scalar>::ContactGraphPlane(const shared_ptr<InputVarList> &varList):ContactGraphBase<Scalar>(varList) {
    
}

/**
 * @brief Class deconstructor
 * @tparam Scalar
 * @param varList
 */
template<typename Scalar>
ContactGraphPlane<Scalar>::~ContactGraphPlane(){
    contact_graphedges.clear();
}

/**
 * @brief build the graph from a list of polygonal meshes
 * @tparam Scalar
 * @param meshes
 * @param atBoundary
 * @param eps
 * @param convexhull : merges faces which on a same plane by its convexhull
 * @return
 */
template<typename Scalar>
bool ContactGraphPlane<Scalar>::buildFromMeshes(const vector<pPolyMesh> &input_meshes,
                                                vector<bool> &atBoundary,
                                                Scalar dist_eps,
                                                Scalar angle_eps,
                                                bool convexhull) {

    // [1] - Scale meshes_input into a united box
    meshes_input = input_meshes;


    // [2] - cluster the faces of input mesh which are on a same plane
    contact_faces.clear();
    if(!clusterFacesofInputMeshes(dist_eps, angle_eps))
        return false;

    tbb::parallel_sort(contact_faces.begin(), contact_faces.end(), [&](const polygonal_face &A, const polygonal_face &B) {
        return A.groupID < B.groupID;
    });

    // [3] - find all pairs of potential face contacts
    contact_pairs.clear();
    listPotentialContacts(atBoundary, angle_eps);

    // [4] - compute the contacts
    contact_graphedges.clear();
    computeContacts();  // note: this function is vectorized with TBB

    // [5] - add nodes into the graph
    nodes.clear();
    buildNodes(atBoundary);

    // [6] - add faces into graph
    edges.clear();
    buildEdges();

    // [7] - assign node ID
    finalize();

    // [8] - simplify contacts
    if(convexhull) computeConvexHullofEdgePolygons();

    return true;
}

/*************************************************
*
*             constructFromPolyMesh methods
*
*************************************************/

/**
 * @brief extract all polygonal faces from the meshes_input and cluster the faces which are on a same plane.
 * @tparam Scalar
 * @param eps
 * @return
 */
template<typename Scalar>
bool ContactGraphPlane<Scalar>::clusterFacesofInputMeshes(Scalar dist_eps, Scalar angle_eps)
{
    //std::set<polygonal_face, plane_contact_compare> planes_set;

    int groupID = 0;
    size_t msize = meshes_input.size();
    vector<polygonal_face> face_groups;

    for (size_t id = 0; id < msize; id++)
    {
        pPolyMesh poly = meshes_input[id];
        if (poly == nullptr)
            return false;

        for (pPolygon face : poly->polyList)
        {
            if (face->vers.size() < 3)
                continue;

            // 1.1) construct plane
            polygonal_face plane;
            Vector3 nrm = face->normal();
            Vector3 center = face->vers[0]->pos;
            plane.nrm = nrm.normalized();

            plane.D = nrm.dot(center);
            plane.partID = id;
            plane.center = center;
            plane.polygon = face;
            plane.eps = dist_eps;

            // 1.2) find groupID
//            typename std::set<polygonal_face, plane_contact_compare>::iterator find_it = planes_set.end();
//            for (int reverse = -1; reverse <= 1; reverse += 2)
//            {
//                polygonal_face tmp_plane = plane;
//                tmp_plane.nrm *= reverse;
//                tmp_plane.D *= reverse;
//
//                find_it = planes_set.find(tmp_plane);
//                if (find_it != planes_set.end())
//                {
//                    plane.groupID = (*find_it).groupID;
//                    break;
//                }
//            }

//            if (find_it == plane_set.end())
//            {
//                plane.groupID = groupID++;
//                plane_set.insert(plane);
//            }

            int find_it = -1;
            for(int gID = 0; gID < face_groups.size(); gID++){
                Vector3 g_nrm = face_groups[gID].nrm;
                Vector3 g_cent = face_groups[gID].center;
                double angle = acos(std::abs(nrm.dot(g_nrm)));
                if(angle < angle_eps){
                    if(std::abs((g_cent - center).dot(g_nrm)) < dist_eps){
                        find_it = gID;
                        plane.groupID = gID;
                        break;
                    }
                }
            }

            if (find_it == -1)
            {
                plane.groupID = groupID++;
                face_groups.push_back(plane);
            }

            contact_faces.push_back(plane);
        }
    }
    return true;
}

/**
 * @brief find all pairs of faces which are on a same plane. Their normals have to be on opposite directions
 * @tparam Scalar
 * @param atBoundary
 */
template<typename Scalar>
void ContactGraphPlane<Scalar>::listPotentialContacts(vector<bool> &atBoundary, double angle_eps)
{
    size_t sta = 0, end = 0;
    while (sta < contact_faces.size())
    {
        for (end = sta + 1; end < contact_faces.size(); end++)
        {
            if (contact_faces[sta].groupID != contact_faces[end].groupID)
            {
                break;
            }
        }
        if (end - sta > 1)
        {
            for (int id = sta; id < end; id++) {
                for (int jd = id + 1; jd < end; jd++)
                {
                    int partI = contact_faces[id].partID;
                    int partJ = contact_faces[jd].partID;

                    Vector3 nrmI = contact_faces[id].nrm.normalized();
                    Vector3 nrmJ = contact_faces[jd].nrm.normalized();
                    double diff_angle = M_PI - acos(nrmI.dot(nrmJ));
                    if (partI != partJ
                        && nrmI.dot(nrmJ) < 0
                        && (!atBoundary[partI] || !atBoundary[partJ])){
                        contact_pairs.push_back(partI < partJ ? pairIJ(id, jd) : pairIJ(jd, id));
                    }

                }
            }
        }
        sta = end;
    }
}

/**
 * @brief Parallel compute contacts.
 *        Though faces in the 'contact_pairs' are on the same plane,
 *        checking whether they are overlap needs the 2D poly-poly intersection tool.
 * @tparam Scalar
 */
template<typename Scalar>
void ContactGraphPlane<Scalar>::computeContacts()
{
    size_t psize = contact_pairs.size();
    contact_graphedges.resize(psize);
    //for (size_t id = 0; id < psize; ++id) // Sequential loop
    tbb::parallel_for(tbb::blocked_range<size_t>(0, psize), [&](const tbb::blocked_range<size_t> &r)
    {
        //for(int id = 0; id < psize; id++)
        for (size_t id = r.begin(); id != r.end(); ++id) {


            int planeI = contact_pairs[id].first;
            int planeJ = contact_pairs[id].second;

            vector<Vector3> polyI = contact_faces[planeI].polygon.lock()->getVertices();
            vector<Vector3> polyJ = contact_faces[planeJ].polygon.lock()->getVertices();

            vector<vector<Vector3>> contactPtLists;

            PolyPolyBoolean<Scalar> ppIntersec(getVarList());

            ppIntersec.computePolygonsIntersection(polyI, polyJ, contactPtLists);
            if (contactPtLists.empty())
            {
                continue;
            }

            vector<pPolygon> contactPolys;
            for (vector<Vector3> contactPtList: contactPtLists)
            {
                if (contactPtList.size() >= 3) {
                    pPolygon contactPoly = make_shared<_Polygon<Scalar>>();
                    contactPoly->setVertices(contactPtList);
                    contactPolys.push_back(contactPoly);
                }
            }

            if (!contactPolys.empty())
            {
                contact_graphedges[id] = make_shared<ContactGraphEdgePlane<Scalar>>(contactPolys, contact_faces[planeI].nrm);
            }
        }
    });
}

/**
 * @brief add each part as a node into the graph
 * @tparam Scalar
 * @param atBoundary
 */
template<typename Scalar>
void ContactGraphPlane<Scalar>::buildNodes(vector<bool> &atBoundary)
{
    for (size_t id = 0; id < meshes_input.size(); id++) {
        Vector3 centroid = meshes_input[id]->centroid();
        Scalar volume = meshes_input[id]->volume();
        pContactGraphNode node = make_shared<ContactGraphNode<Scalar>>(atBoundary[id], centroid, centroid, volume);
        addNode(node);
    }
}

/**
 * @brief for all pairs of potential contact faces,
 *        if their contact_graphedges is not empty, then we add this edge into the graph
 * @tparam Scalar
 */
template<typename Scalar>
void ContactGraphPlane<Scalar>::buildEdges() {

    for (size_t id = 0; id < contact_pairs.size(); id++)
    {
        pContactGraphEdge edge = contact_graphedges[id];
        if (edge != nullptr) {
            int planeI = contact_pairs[id].first;
            int planeJ = contact_pairs[id].second;
            int partI = contact_faces[planeI].partID;
            int partJ = contact_faces[planeJ].partID;
            addContact(nodes[partI], nodes[partJ], edge);
        }
    }
}

/**
 * @brief simplified the contact between two nodes, only compute their convex-hull
 * @tparam Scalar
 */
template<typename Scalar>
void ContactGraphPlane<Scalar>::computeConvexHullofEdgePolygons()
{
    contact_graphedges.erase(std::remove(contact_graphedges.begin(), contact_graphedges.end(), nullptr), contact_graphedges.end());

    std::sort(contact_graphedges.begin(), contact_graphedges.end(),
              [&](      shared_ptr<ContactGraphEdgePlane<Scalar>> e0,
                        shared_ptr<ContactGraphEdgePlane<Scalar>> e1){
        if(e0->partIDA < e1->partIDA)
            return true;
        if(e0->partIDA > e1->partIDA)
            return false;

        if(e0->partIDB < e1->partIDB)
            return true;
        if(e0->partIDB > e1->partIDB)
            return false;

        if (e0->normal[0] - e1->normal[0] < -FLOAT_ERROR_LARGE)
            return true;
        if (e0->normal[0] - e1->normal[0] > FLOAT_ERROR_LARGE)
            return false;

        if (e0->normal[1] - e1->normal[1] < -FLOAT_ERROR_LARGE)
            return true;
        if (e0->normal[1] - e1->normal[1] > FLOAT_ERROR_LARGE)
            return false;

        if (e0->normal[2] - e1->normal[2] < -FLOAT_ERROR_LARGE)
            return true;
        if (e0->normal[2] - e1->normal[2] > FLOAT_ERROR_LARGE)
            return false;

        return false;
    });


    edges.clear();
    size_t sta = 0, end = 0;

    while (sta < contact_graphedges.size()) {
        for (end = sta + 1; end < contact_graphedges.size(); end++) {
            if(!contact_graphedges[sta]->check_on_same_plane(contact_graphedges[end])){
                break;
            }
        }

        pContactGraphEdge edge;
        if(end > sta + 1)
        {
            vector<Vector3> corner_points;
            vector<pPolygon> polygons;
            for(int id = sta; id < end; id++)
            {
                vector<Vector3> poly_points = contact_graphedges[id]->points;
                corner_points.insert(corner_points.end(), poly_points.begin(), poly_points.end());
                polygons.insert(polygons.begin(), contact_graphedges[id]->polygons.begin(), contact_graphedges[id]->polygons.end());
            }

            Vector3 normal = contact_graphedges[sta]->normal, x_axis, y_axis;

            vector<Vector3> convexhull_points;
            ConvexHull2D<Scalar> convexhull_solver;
            convexhull_solver.compute(corner_points, normal, convexhull_points);

            edge = make_shared<ContactGraphEdgeBase<Scalar>>(polygons, convexhull_points, contact_graphedges[sta]->normal);
        }
        else{
            edge = make_shared<ContactGraphEdgeBase<Scalar>>(contact_graphedges[sta]->polygons, contact_graphedges[sta]->normal);
        }

        edge->partIDA = contact_graphedges[sta]->partIDA;
        edge->partIDB = contact_graphedges[sta]->partIDB;
        edges.push_back(edge);
        sta = end;
    }
}

// No need to call this TemporaryFunction() function,
// it's just to avoid link error.
void TemporaryFunction_ContactGraph ()
{
    shared_ptr<InputVarList> varList;
    ContactGraphPlane<double> contactGraph(varList);
    vector<shared_ptr<PolyMesh<double>>> polyMesh;
    vector<bool> atBoundary;
    contactGraph.buildFromMeshes(polyMesh, atBoundary);
    contactGraph.mergeNode(nullptr, nullptr);
    contactGraph.getContactMesh(polyMesh.front());
}

template class ContactGraphPlane<double>;
template class ContactGraphPlane<float>;
