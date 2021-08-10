//
// Created by ziqwang on 14.01.19.
//

#ifndef TOPOLOCKCREATOR_CONTACTGRAPHEDGE_H
#define TOPOLOCKCREATOR_CONTACTGRAPHEDGE_H

#include <Eigen/Dense>
#include "Utility/HelpDefine.h"
#include <Mesh/Polygon.h>

/*!
 * \brief: Stores which parts are contacted and the contact's geometry
 */
template<typename Scalar>
class ContactGraphEdgeBase
{
public:

    typedef shared_ptr<_Polygon<Scalar>> pPolygon;

    typedef Matrix<Scalar, 3, 1> Vector3;

public:
    vector<Vector3> points;  // a list which polygons are simple and closed (better to be convex)
    vector<shared_ptr<_Polygon<Scalar>>> polygons; //for rendering
    vector<Vector3> normals; // contact normal from partIDA to partIDB

public:

    ContactGraphEdgeBase(const shared_ptr<_Polygon<Scalar>> &_polygon, const Vector3 &_normal)
    {
        for(int id = 0; id < _polygon->size(); id++){
            points.push_back(_polygon->pos(id));
            normals.push_back(_normal);
        }

        polygons.push_back(_polygon);
    }


    ContactGraphEdgeBase(const vector<pPolygon> &_polygons, const Vector3 &_normal)
    {
        for(pPolygon poly: _polygons){
            for(int id = 0; id < poly->size(); id++){
                points.push_back(poly->pos(id));
                normals.push_back(_normal);
            }

            polygons.push_back(poly);
        }
    }

    ContactGraphEdgeBase(const vector<Vector3> &_points, const vector<Vector3> &_normals)
    {
        for(int id = 0; id < _points.size(); id++){
            points.push_back(_points[id]);
            normals.push_back(_normals[id]);
        }
    }


    ContactGraphEdgeBase(const vector<pPolygon> &_polygons, const vector<Vector3> _points, const Vector3 &_normal)
    {
        for(pPolygon poly: _polygons){
            polygons.push_back(poly);
        }

        for(int id = 0; id < _points.size(); id++){
            points.push_back(_points[id]);
            normals.push_back(_normal);
        }
    }

    ContactGraphEdgeBase(const vector<pPolygon> &_polygons, const vector<Vector3> _points, const vector<Vector3> &_normals)
    {
        polygons = _polygons;
        points = _points;
        normals = _normals;
    }


    /*!
     * \return the contact normal starts from partID
     */
    Vector3 getContactNormal(int partID, int pointID){
        if(partIDA == partID)
        {
            return normals[pointID];
        }
        else if(partIDB == partID){
            return normals[pointID] * (-1.0);
        }
        return Vector3(0, 0, 0);
    }

    void get_norm_fric_for_block(int partID, int pointID, Vector3 &normal, Vector3 &ufric, Vector3 &vfric)
    {
        // the contact normal points from partA to partB
        // when considering contact force acting on part A,
        // the force direction is from partB to part A
        // therefore, here has a -1.
        normal = getContactNormal(partID, pointID) * (-1.0);
        ufric = Vector3(1, 0, 0).cross(normal);
        if((ufric).norm() < FLOAT_ERROR_SMALL){
            ufric = Vector3(0, 1, 0).cross(normal);
        }
        ufric.normalize();
        vfric = normal.cross(ufric).normalized();
        return;
    }

    size_t size(){
        return points.size();
    }

public: //Automatic Generate

    int partIDA; // Static ID for start node of this edge

    int partIDB; // Static ID for end node of this edge
};


#endif //TOPOLOCKCREATOR_CONTACTGRAPHEDGE_H
