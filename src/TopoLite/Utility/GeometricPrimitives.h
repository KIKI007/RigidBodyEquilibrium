///////////////////////////////////////////////////////////////
//
// Utility/HelpStruct.h
//
//   Common Structures
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 01/Aug/2018
//
//
///////////////////////////////////////////////////////////////

#ifndef GeometricPrimitives_H
#define GeometricPrimitives_H

#include "TopoLite/Utility/HelpDefine.h"
#include <nlohmann/json.hpp>
#include <vector>
#include <Eigen/Dense>
#include <cmath>

using Eigen::Matrix;
using Eigen::Vector3d;
using Eigen::Vector3f;

using namespace std;

// Point position w.r.t a plane
#define POINT_PLANE_UNKWONN          -1
#define POINT_PLANE_INTERSECT         0
#define POINT_PLANE_POSITIVE_SIDE     1
#define POINT_PLANE_NEGATIVE_SIDE     2

// Line position w.r.t a plane
#define LINE_PLANE_UNKWONN           -1
#define LINE_PLANE_INTERSECT          0
#define LINE_PLANE_POSITIVE_SIDE      1
#define LINE_PLANE_NEGATIVE_SIDE      2

//
#define LINE_LINE_NOINTERSECTION    0
#define LINE_LINE_INTERSECTION      1
#define LINE_LINE_COLLINEAR          2

// Face position w.r.t a plane
#define FACE_PLANE_UNKWONN           -1
#define FACE_PLANE_INTERSECT          0
#define FACE_PLANE_POSITIVE_SIDE      1
#define FACE_PLANE_NEGATIVE_SIDE      2

////////////////////////////////////////////
// 3D Point
////////////////////////////////////////////

// 3D Point (with normal)
template <typename Scalar>
struct Point
{
    typedef Matrix<Scalar,3, 1> Vector3;

    Vector3 pos;     // Point position
    Vector3 nor;     // Point normal
    Vector3 color;   // Point color (if available)

    Scalar curv;       // Point curvature
	int dist;         // Distance to the object surface

	Point(){
	    pos = Vector3(0, 0, 0);
        nor = Vector3(0, 0, 0);
        color = Vector3(0, 0, 0);
        curv = 0;
        dist = 0;
	}

	Point & operator=(const Point &pt)
	{
		if( this == &pt )
			return *this;

		this->pos   = pt.pos;
		this->nor   = pt.nor;

		this->color = pt.color;
		this->curv  = pt.curv;

		return *this;
	};
};

////////////////////////////////////////////
// 3D Vertex
////////////////////////////////////////////

template <typename Scalar>
class VPoint
{
public:

    typedef Matrix<Scalar, 3, 1> Vector3;

public:

    Vector3 pos;				// vertex's postion (X,Y,Z)

    Vector3 nrm;

    int verID;

public:
    VPoint()
    {
        verID = -1;
        pos = Vector3(0, 0 , 0);
    };

    VPoint(const Vector3 &_pos)
    {
        pos   = _pos;
        verID = -1;
        nrm = Vector3(0, 0 , 0);
    };

    VPoint(const VPoint<Scalar> &v){
        pos = v.pos;
        nrm = v.nrm;
        verID = v.verID;
    }
};


template <typename Scalar>
class VTex
{
public:

    typedef Matrix<Scalar, 2 ,1> Vector2;

public:

    Vector2 texCoord;				// vertex's postion (X,Y,Z)

    int texID;

public:
    VTex()
    {
        texID = -1;
    };

    VTex(const Vector2& _texCoord)
    {
        texCoord = _texCoord;
        texID = -1;
    };
};


////////////////////////////////////////////
// 3D Line
////////////////////////////////////////////

template <typename Scalar>
struct Line
{
    typedef Matrix<Scalar,3, 1> Vector3;
    typedef Matrix<Scalar,2, 1> Vector2;

    Vector3 point1;
	Vector3 point2;

	Line(){
	    point1 = Vector3(0 ,0 ,0);
        point2 = Vector3(0 ,0 ,0);
	}

	Line(Vector3 pt1, Vector3 pt2){
	    point1 = pt1;
	    point2 = pt2;
	}

    Line(Vector2 pt1, Vector2 pt2){
        point1 = Vector3(pt1.x(), pt1.y(), 0);
        point2 = Vector3(pt2.x(), pt2.y(), 0);
    }

public:

    Vector3d direction(){
	    return (point2 - point1).normalized();
	}

    //https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
    Scalar distance2(Eigen::Vector2d pt){
	    Scalar x = pt.x();
	    Scalar y = pt.y();
	    Scalar x1 = point1.x();
	    Scalar y1 = point1.y();
	    Scalar x2 = point2.x();
	    Scalar y2 = point2.y();
	    
        Scalar A = x - x1;
        Scalar B = y - y1;
        Scalar C = x2 - x1;
        Scalar D = y2 - y1;

        Scalar dot = A * C + B * D;
        Scalar len_sq = C * C + D * D;
        Scalar param = -1;
        if (len_sq != 0) //in case of 0 length line
            param = dot / len_sq;

        Scalar xx, yy;

        if (param < 0) {
            xx = x1;
            yy = y1;
        }
        else if (param > 1) {
            xx = x2;
            yy = y2;
        }
        else {
            xx = x1 + param * C;
            yy = y1 + param * D;
        }

        Scalar dx = x - xx;
        Scalar dy = y - yy;
        return std::sqrt(dx * dx + dy * dy);
    }

    double distance3(Eigen::Vector3d pt){
	    if((point2 - point1).norm() < FLOAT_ERROR_SMALL){
	        return 0;
	    }
	    else{
            return ((pt - point1).cross(point2 - point1)).norm() / (point2 - point1).norm();
        }
	}

    int checkIntersection(Line<Scalar> &lineB){
	    double dist1 = lineB.distance3(point1);
	    double dist2 = lineB.distance3(point2);
	    if(dist1 < FLOAT_ERROR_SMALL && dist2 < FLOAT_ERROR_SMALL)
	    {
	        double norm = pow((point2 - point1).norm(), 2);
	        double tB1 = (lineB.point1 - point1).dot(point2 - point1) / norm;
	        double tB2 = (lineB.point2 - point1).dot(point2 - point1) / norm;
	        if(tB2 < tB1) std::swap(tB1, tB2);
            if(tB2 <= FLOAT_ERROR_LARGE || tB1 >= 1 - FLOAT_ERROR_LARGE) return LINE_LINE_COLLINEAR;
            return LINE_LINE_INTERSECTION;
	    }
	    else{
	        return LINE_LINE_NOINTERSECTION;
	    }
	}

};


////////////////////////////////////////////
// 3D Plane
////////////////////////////////////////////

template <typename Scalar>
struct Plane
{
    typedef Matrix<Scalar,3, 1> Vector3;

    Vector3 point;
	Vector3 normal;

    Plane(){
        point = Vector3(0, 0, 0);
        normal = Vector3(0, 0, 0);
    }

	Plane & operator=(const Plane &plane);

	Scalar computePtPtDistance(Vector3 tagtPt);
	int checkAtSameHalfSpace(Vector3 tagtPt) const ;

	int checkLnPlnIntersec(Line<Scalar> line);
	int computeLnLnIntersec(Line<Scalar> line, Vector3 &crossPt);
};

template<typename Scalar>
Plane<Scalar> & Plane<Scalar>::operator=(const Plane<Scalar> &plane)
{
    if( this == &plane )
        return *this;

    this->point  = plane.point;
    this->normal = plane.normal;

    this->radius = plane.radius;

    return *this;
}

template<typename Scalar>
Scalar Plane<Scalar>::computePtPtDistance(Vector3 tagtPt)
{
    Vector3 tagtvec = tagtPt - point;

    Scalar dotP = tagtvec.dot(normal);
    Scalar dist = std::abs(dotP);

    return dist;
}


template<typename Scalar>
int Plane<Scalar>::checkAtSameHalfSpace(Vector3 tagtPt) const
{
    Vector3 vec = tagtPt - point;
    Scalar dotP = normal.dot(vec);

    if ( std::abs(dotP) < FLOAT_ERROR_SMALL)
        return POINT_PLANE_INTERSECT;       // Note: may need to tune this small threshold
    else if ( dotP >  0 )
        return POINT_PLANE_POSITIVE_SIDE;
    else if ( dotP <  0 )
        return POINT_PLANE_NEGATIVE_SIDE;

    return POINT_PLANE_UNKWONN;
}

template<typename Scalar>
int Plane<Scalar>::checkLnPlnIntersec(Line<Scalar> line)
{
    int pt1State = checkAtSameHalfSpace(line.point1);
    int pt2State = checkAtSameHalfSpace(line.point2);

    if ( pt1State == POINT_PLANE_POSITIVE_SIDE &&
         pt2State == POINT_PLANE_POSITIVE_SIDE )
        return LINE_PLANE_POSITIVE_SIDE;

    else
    if ( pt1State == POINT_PLANE_NEGATIVE_SIDE &&
         pt2State == POINT_PLANE_NEGATIVE_SIDE )
        return LINE_PLANE_NEGATIVE_SIDE;

    else
        return LINE_PLANE_INTERSECT;
}

template<typename Scalar>
int Plane<Scalar>::computeLnLnIntersec(Line<Scalar> line, Vector3 &crossPt)
{
    // Check if the line intersects the plane
    int state = checkLnPlnIntersec( line );
    if ( state != LINE_PLANE_INTERSECT )
    {
        //printf("Warning: Line does not intersect with the plane. \n");
        return state;
    }

    // Compute the intersected point between the line and the plane
    Vector3 rayOrg = line.point1;
    Vector3 rayDir = (line.point2-line.point1) / (line.point2-line.point1).norm();
    Vector3 tempVec = point - rayOrg;
    Scalar m = (normal.dot(tempVec)) / (normal.dot(rayDir));

    crossPt[0] = rayOrg[0] + m * rayDir[0];
    crossPt[1] = rayOrg[1] + m * rayDir[1];
    crossPt[2] = rayOrg[2] + m * rayDir[2];

    return state;
}


////////////////////////////////////////////
// 3D Box
////////////////////////////////////////////

template <typename Scalar>
struct Box 
{
    typedef Matrix<Scalar,3, 1> Vector3;
    typedef Matrix<Scalar,2, 1> Vector2;
public:
    //storage
    Vector3 minPt;
    Vector3 maxPt;

public:
    //compute
    Vector3 cenPt;
    Vector3 size;

public:
	Box();
	Box(const Line<Scalar> &line);
	Box(const vector<Vector3> &pts);
    Box(const vector<Vector2> &pts);
	Box(const Box<Scalar> &b0, const Box<Scalar> &b1);
	Box & operator=(const Box &box);
	void print();

	void computeCenter();
	void computeSize();

	vector<Vector2> convert2DPolyList();

	void executeTransform(Vector3 transVec, Vector3 scale);

	//if this box degenerates into a quad plane, compute its area.
	Scalar computeQuadArea();
};

template<typename Scalar>
Box<Scalar>::Box()
{
    minPt = Vector3(0, 0, 0);
    maxPt = Vector3(0, 0, 0);
    cenPt = Vector3(0, 0, 0);
    size = Vector3(0, 0, 0);
}

template<typename Scalar>
Box<Scalar>::Box(const Line<Scalar> &line)
{
    for(size_t id = 0; id < 3; id++){
        minPt[id] = std::min(line.point1[id], line.point2[id]);
        maxPt[id] = std::max(line.point1[id], line.point2[id]);
    }

    cenPt = (minPt + maxPt)/2;
    size = (maxPt - minPt);
}

template<typename Scalar>
Box<Scalar>::Box(const vector<Vector3> &pts){

    minPt = Vector3(numeric_limits<Scalar>::max(), numeric_limits<Scalar>::max(), numeric_limits<Scalar>::max());
    maxPt = Vector3(numeric_limits<Scalar>::lowest(), numeric_limits<Scalar>::lowest(), numeric_limits<Scalar>::lowest());

    for(int kd = 0; kd < pts.size(); kd++){
        for(size_t id = 0; id < 3; id++)
        {
            minPt[id] = std::min(pts[kd][id], minPt[id]);
            maxPt[id] = std::max(pts[kd][id], maxPt[id]);
        }
    }

    cenPt = (minPt + maxPt)/2;
    size = (maxPt - minPt);
}

template<typename Scalar>
Box<Scalar>::Box(const vector<Vector2> &pts){

    minPt = Vector3(numeric_limits<Scalar>::max(), numeric_limits<Scalar>::max(), 0);
    maxPt = Vector3(numeric_limits<Scalar>::lowest(), numeric_limits<Scalar>::lowest(), 0);

    for(int kd = 0; kd < pts.size(); kd++){
        for(size_t id = 0; id < 2; id++)
        {
            minPt[id] = std::min(pts[kd][id], minPt[id]);
            maxPt[id] = std::max(pts[kd][id], maxPt[id]);
        }
    }

    cenPt = (minPt + maxPt)/2;
    size = (maxPt - minPt);
}

template<typename Scalar>
Box<Scalar>::Box(const Box<Scalar> &b0, const Box<Scalar> &b1){

    for(size_t id = 0; id < 3; id++)
    {
        minPt[id] = std::min(b0.minPt[id], b1.minPt[id]);
        maxPt[id] = std::max(b0.maxPt[id], b1.maxPt[id]);
    }

    cenPt = (minPt + maxPt)/2;
    size = (maxPt - minPt);
}


template<typename Scalar>
Box<Scalar> & Box<Scalar>::operator=(const Box &box)
{
    if( this == &box )
        return *this;

    this->minPt = box.minPt;
    this->maxPt = box.maxPt;
    this->cenPt = box.cenPt;
    this->size = box.size;

    return *this;
}

template<typename Scalar>
void Box<Scalar>::print()
{
    printf("Box: [%7.3f  %7.3f  %7.3f]      [%7.3f  %7.3f  %7.3f] \n", minPt.x(), minPt.y(), minPt.z(), maxPt.x(), maxPt.y(), maxPt.z());
}

template<typename Scalar>
void Box<Scalar>::computeCenter()
{
    cenPt = 0.5f * ( minPt + maxPt );
}

template<typename Scalar>
void Box<Scalar>::computeSize()
{
    size = maxPt - minPt;
}

template<typename Scalar>
void Box<Scalar>::executeTransform(Vector3 transVec, Vector3 scale)
{
    minPt.x() *= scale.x();  minPt.y() *= scale.y();  minPt.z() *= scale.z();
    maxPt.x() *= scale.x();  maxPt.y() *= scale.y();  maxPt.z() *= scale.z();
    cenPt.x() *= scale.x();  cenPt.y() *= scale.y();  cenPt.z() *= scale.z();
    size.x() *= scale.x();   size.y() *= scale.y();   size.z() *= scale.z();

    minPt += transVec;
    maxPt += transVec;
    cenPt += transVec;
}

template<typename Scalar>
Scalar Box<Scalar>::computeQuadArea()
{
    Vector3 dimen = maxPt - minPt;
    Scalar quadArea = 0;

    if      ( std::abs(dimen.x()) < FLOAT_ERROR_SMALL && dimen.y()  > 0 &&  dimen.z()  > 0 )
        quadArea = dimen.y() * dimen.z();  // y-z plane quad
    else if ( dimen.x()  > 0 && std::abs(dimen.y()) < FLOAT_ERROR_SMALL &&  dimen.z()  > 0 )
        quadArea = dimen.x() * dimen.z();  // x-z plane quad
    else if ( dimen.x()  > 0 && dimen.y()  > 0 &&  std::abs(dimen.z()) < FLOAT_ERROR_SMALL )
        quadArea = dimen.x() * dimen.y();  // x-y plane quad
    else
        printf("Warning: The box is not degenerated into a quad. \n");

    return quadArea;
}

template<typename Scalar>
vector<Matrix<Scalar, 2, 1>> Box<Scalar>::convert2DPolyList() {
    vector<Eigen::Vector2d> poly;
    poly.push_back(Eigen::Vector2d(minPt[0], minPt[1]));
    poly.push_back(Eigen::Vector2d(maxPt[0], minPt[1]));
    poly.push_back(Eigen::Vector2d(maxPt[0], maxPt[1]));
    poly.push_back(Eigen::Vector2d(minPt[0], maxPt[1]));
    return poly;
}

////////////////////////////////////////////
// Box With Coordinate Frame
////////////////////////////////////////////

template<typename Scalar>
struct BoxFrame : public Box<Scalar>{
public:
    typedef Matrix<Scalar,3, 1> Vector3;

    Vector3 origin, xaxis, yaxis, zaxis;

public:

    BoxFrame():Box<Scalar>(){
        origin = Vector3(0, 0, 0);
        xaxis = Vector3(1, 0, 0);
        yaxis = Vector3(0, 1, 0);
        zaxis = Vector3(0, 0, 1);
    }

    BoxFrame(const Box<Scalar> &box,
             Vector3 _origin,
             Vector3 _xaxis,
             Vector3 _yaxis,
             Vector3 _zaxis): Box<Scalar>(box){
        origin = _origin;
        xaxis = _xaxis;
        yaxis = _yaxis;
        zaxis = _zaxis;
    }
};

////////////////////////////////////////////
// 3D Triangle
////////////////////////////////////////////

template <typename Scalar>
struct Triangle
{
public:
    typedef Matrix<Scalar,3, 1> Vector3;

    typedef Matrix<Scalar,2, 1> Vector2;

public:

    Vector3 v[3];

    Vector3 vf[3];

	int vIndices[3];       // Index of each vertex

	bool edge_at_boundary[3];
public:

    void clear(){
        for(size_t kd = 0; kd < 3; kd++){
            edge_at_boundary[kd] = false;
            v[kd] = Vector3(0, 0, 0);
            vf[kd] = Vector3(0, 0, 0);
        }
    }

    Triangle(){
        clear();
    }

    Triangle(Vector3 _v0, Vector3 _v1, Vector3 _v2)
    {
        clear();
        v[0] = _v0;
        v[1] = _v1;
        v[2] = _v2;
    }

    Triangle(Vector2 _v0, Vector2 _v1, Vector2 _v2)
    {
        clear();
        v[0].x() = _v0.x(); v[0].y() = _v0.y(); v[0].z() = 0;
        v[1].x() = _v1.x(); v[1].y() = _v1.y(); v[1].z() = 0;
        v[2].x() = _v2.x(); v[2].y() = _v2.y(); v[2].z() = 0;
    }

    Triangle(Vector3 _v0, Vector3 _v1, Vector3 _v2, bool b0, bool b1, bool b2)
    {
        clear();
        v[0] = _v0;
        v[1] = _v1;
        v[2] = _v2;
        edge_at_boundary[0] = b0;
        edge_at_boundary[1] = b1;
        edge_at_boundary[2] = b2;
    }

public:

	void init(Vector3 _v0, Vector3 _v1, Vector3 _v2);

	Triangle & operator=(const Triangle &tri);

	bool checkEqual(const Triangle<Scalar> &tri);

	void print();

	Vector3 computeBBoxMinPt();

	Vector3 computeBBoxMaxPt();

    Vector3 computeCenter();

	Scalar computeArea();

    Scalar computeSignedArea(); //for 2D triangle

	Vector3 computeNormal();

    void correctNormal(Vector3 tagtNormal);
};

template<typename Scalar>
void Triangle<Scalar>::init(Vector3 _v0, Vector3 _v1, Vector3 _v2)
{
    v[0] = _v0;
    v[1] = _v1;
    v[2] = _v2;
}

template<typename Scalar>
Triangle<Scalar> & Triangle<Scalar>::operator=(const Triangle &tri)
{
    if( this == &tri )
        return *this;

    for (int i=0; i<3; i++)
    {
        this->v[i] = tri.v[i];
        this->vIndices[i] = tri.vIndices[i];
    }
    return *this;
}

template<typename Scalar>
bool Triangle<Scalar>::checkEqual(const Triangle<Scalar> &tri)
{
    if( this->v[0] == tri.v[0] &&
        this->v[1] == tri.v[1] &&
        this->v[2] == tri.v[2] )
    {
        return true;
    }
    else
    {
        return false;
    }
}

template<typename Scalar>
void Triangle<Scalar>::print()
{
    printf("v0: [%.12f %.12f %.12f] \n", v[0].x(), v[0].y(), v[0].z());
    printf("v1: [%.12f %.12f %.12f] \n", v[1].x(), v[1].y(), v[1].z());
    printf("v2: [%.12f %.12f %.12f] \n", v[2].x(), v[2].y(), v[2].z());
    printf("\n");
}

template<typename Scalar>
Matrix<Scalar, 3, 1> Triangle<Scalar>::computeBBoxMinPt()
{
    Vector3 bboxMinPt;

    bboxMinPt.x() = _MIN(v[0].x(), _MIN(v[1].x(), v[2].x()));
    bboxMinPt.y() = _MIN(v[0].y(), _MIN(v[1].y(), v[2].y()));
    bboxMinPt.z() = _MIN(v[0].z(), _MIN(v[1].z(), v[2].z()));

    return bboxMinPt;
}
template<typename Scalar>
Matrix<Scalar, 3, 1> Triangle<Scalar>::computeBBoxMaxPt()
{
    Vector3 bboxMaxPt;

    bboxMaxPt.x() = _MAX(v[0].x(), _MAX(v[1].x(), v[2].x()));
    bboxMaxPt.y() = _MAX(v[0].y(), _MAX(v[1].y(), v[2].y()));
    bboxMaxPt.z() = _MAX(v[0].z(), _MAX(v[1].z(), v[2].z()));

    return bboxMaxPt;
}

template<typename Scalar>
Matrix<Scalar, 3, 1> Triangle<Scalar>::computeCenter()
{
    Vector3 center = (v[0] + v[1] + v[2]) / 3.0;
    return center;
}

template<typename Scalar>
Scalar Triangle<Scalar>::computeArea()
{
    Vector3 normal  = (v[1] - v[0]).cross(v[2] - v[0]);
    Scalar area  = 0.5f * normal.norm();
    return area;
}

//https://www.mn.uio.no/math/english/people/aca/michaelf/papers/wach_mv.pdf
template<typename Scalar>
Scalar Triangle<Scalar>::computeSignedArea()
{
    Matrix<Scalar, 3, 3> A;
    A << 1, 1, 1,
    v[0].x(), v[1].x(), v[2].x(),
    v[0].y(), v[1].y(), v[2].y();
    return A.determinant() / 2;
}


template<typename Scalar>
Matrix<Scalar, 3, 1> Triangle<Scalar>::computeNormal()
{
    Vector3 tempNor = (v[1] - v[0]).cross(v[2] - v[0]);  // Assume the vertices are saved in counter-clockwise
    Scalar tempNorLen = tempNor.norm();
    Vector3 normal;
    if ( tempNorLen > FLOAT_ERROR_SMALL )
    {
        normal = tempNor / tempNorLen;
    }
    else{
        normal = Vector3(1,0,0);     // Note: this default vector also can be others
    }
    return normal;
}

template<typename Scalar>
void Triangle<Scalar>::correctNormal(Vector3 tagtNormal)
{
    // Compute initial normal
    Vector3 normal = computeNormal();

    // Rearrange vertex order if needed
    Scalar dotp = normal.dot(tagtNormal);
    if ( dotp < 0 )
    {
        Vector3 triVers[3];
        for (int i=0; i<3; i++)
        {
            triVers[i] = v[i];
        }

        v[0] = triVers[0];
        v[1] = triVers[2];
        v[2] = triVers[1];
    }
}

////////////////////////////////////////////
// For computing part geometry and mobility
////////////////////////////////////////////

template <typename Scalar>
struct OrientPoint
{
    typedef Matrix<Scalar, 3, 1> Vector3;
    typedef Matrix<Scalar, 2, 1> Vector2;

    // for geometry generation
    Vector3 point;
    Vector3 normal;             // normed vector
    Vector3 rotation_axis;
    Vector3 rotation_base;

	// for optimization
	Scalar rotation_angle;     // always positive
	int tiltSign;              // Flag that indicates the direction to tilt the normal (possible values are {-1, 1})
    int oriptID;               // the index of the oriented point in the whole structure
    Vector2 tilt_range;	       // the lower and upper bound of tilt angle such that the structure have valid geometry.
    Vector2 sided_range;	   // for debug, show the side range

    //for storage
    Vector3 edge_vec;
    Vector3 cross_normal;
public:

    OrientPoint(const nlohmann::json &oript_json){
        parse(oript_json);
    }

	OrientPoint(Vector3 _point, Vector3 _normal)
	{
		point  = _point;
		normal = _normal;
		rotation_base = _normal;
		rotation_axis = Vector3(0, 0, 0);
		rotation_angle = 0;
		tiltSign = TILT_SIGN_NONE;
		oriptID = -1;
		tilt_range[0] = 0;
		tilt_range[1] = 180;
	}

	OrientPoint(Vector3 _point, Vector3 _normal, Vector3 _axis)
	{
		point  = _point;
		normal = _normal;
		rotation_base = _normal;
		rotation_axis = _axis;
		rotation_angle = 0;
		tiltSign = TILT_SIGN_NONE;
		oriptID = -1;
		tilt_range[0] = 0;
		tilt_range[1] = 180;
	};

    /**
        @brief: Compute vector rotation
        @param[in] normal based vector
        @param[in] rotAxis rotation axis
        @param[in] rotAngle rotation angle
        @return vector after rotation
    */
    static Vector3 rotateVecAroundAxis(Vector3 normal, Vector3 rotAxis, Scalar rotAngle)
    {
        rotAngle = rotAngle / 180 * M_PI;
        Eigen::Matrix<Scalar, 3, 3> rotationMat, crossprodMat;
        crossprodMat << 0,             -rotAxis.z(),   rotAxis.y(),
                        rotAxis.z(),    0,            -rotAxis.x(),
                       -rotAxis.y(),    rotAxis.x(),   0;
        rotationMat = Eigen::Matrix<Scalar, 3, 3>::Identity() * std::cos(rotAngle) + crossprodMat * std::sin(rotAngle);
        return rotationMat * normal;
    }

    /**
        Update rotation_angle (always positive) + define normal
    */
    void updateAngle(Scalar _angle)
	{
		rotation_angle = std::abs(_angle);
        normal = rotateVecAroundAxis(rotation_base, rotation_axis, rotation_angle * tiltSign);
	}

	void print()
	{
		printf("point: [%6.3f %6.3f %6.3f]   normal: [%6.3f %6.3f %6.3f] \n", point[0], point[1], point[2], normal[0], normal[1], normal[2]);
		printf("rotation_axis: [%6.3f %6.3f %6.3f]   rotation_angle: [%6.3f] \n", rotation_axis[0], rotation_axis[1], rotation_axis[2], rotation_angle);
	};

    nlohmann::json dump(){
        nlohmann::json oript_json;
        oript_json["middle point"] = {point.x(), point.y(), point.z()};
        oript_json["face normal"] = {normal.x(), normal.y(), normal.z()};
        oript_json["rotation axis"] = {rotation_axis.x(), rotation_axis.y(), rotation_axis.z()};
        oript_json["rotation base"] = {rotation_base.x(), rotation_base.y(), rotation_base.z()};
        oript_json["tilting range"] = {tilt_range.x(), tilt_range.y()};

        oript_json["rotation angle"] = rotation_angle;
        oript_json["tilting sign"] = tiltSign;
        oript_json["globalID"] = oriptID;
        return oript_json;
    }

    void parse(const nlohmann::json &oript_json){
        point = Vector3(((vector<Scalar>)oript_json["middle point"]).data());
        normal = Vector3(((vector<Scalar>)oript_json["face normal"]).data());
        rotation_axis = Vector3(((vector<Scalar>)oript_json["rotation axis"]).data());
        rotation_base = Vector3(((vector<Scalar>)oript_json["rotation base"]).data());
        tilt_range = Vector2(((vector<Scalar>)oript_json["tilting range"]).data());

        rotation_angle = (Scalar)oript_json["rotation angle"];
        tiltSign = (int)oript_json["tilting sign"];
        oriptID = (int)oript_json["globalID"];
    }
};

template <typename Scalar>
struct HypVertex
{
    typedef Matrix<Scalar, 3, 1> Vector3;

public:
	int verID;
	Vector3 point;
	int planeIDs[3];
};

template <typename Scalar>
struct HypEdge
{
    typedef Matrix<Scalar, 3, 1> Vector3;

public:
	int edgeID;
	Vector3 point;
	Vector3 normal;

	int planeIDs[2];
};

template <typename Scalar>
struct HypPlane
{
    typedef Matrix<Scalar, 3, 1> Vector3;

public:
	int planeID;
	Vector3 point;
	Vector3 normal;

public:
	Scalar getD(){return normal.dot(point);}

	Scalar radius;             // For rendering a finite plane in 3D
};


#endif