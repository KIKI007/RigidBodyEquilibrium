//
// Created by ziqwang on 03.08.20.
//

#ifndef TOPOLITE_CONTACTGRAPHEDGEPLANE_H
#define TOPOLITE_CONTACTGRAPHEDGEPLANE_H

#include "ContactGraphEdgeBase.h"

template<typename Scalar>
class ContactGraphEdgePlane: public ContactGraphEdgeBase<Scalar>{
public:

    typedef shared_ptr<_Polygon<Scalar>> pPolygon;

    typedef Matrix<Scalar, 3, 1> Vector3;

public:

    using ContactGraphEdgeBase<Scalar>::partIDA;

    using ContactGraphEdgeBase<Scalar>::partIDB;

public:

    Vector3 normal;

public:

    ContactGraphEdgePlane(pPolygon &_polygon, Vector3 &_normal): ContactGraphEdgeBase<Scalar>(_polygon, _normal)
    {
        normal = _normal;
    }


    ContactGraphEdgePlane(vector<pPolygon> &_polygons, Vector3 &_normal) : ContactGraphEdgeBase<Scalar>(_polygons, _normal)
    {
        normal = _normal;
    }

public:

    bool check_on_same_plane(shared_ptr<ContactGraphEdgePlane<Scalar>> e){
        //always suppose partIDA < partIDB

        if(partIDA != e->partIDA || partIDB != e->partIDB){
            return false;
        }

        if(normal.cross(e->normal).norm() < FLOAT_ERROR_LARGE && normal.dot(e->normal) >= 0){
            return true;
        }

        return false;
    }

};

#endif //TOPOLITE_CONTACTGRAPHEDGEPLANE_H
