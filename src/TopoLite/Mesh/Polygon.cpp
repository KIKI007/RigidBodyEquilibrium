///////////////////////////////////////////////////////////////
//
// Polygon.cpp
//
//   3D Polygon (vertices may or may not be co-planar)
//
// by Peng SONG  ( songpenghit@gmail.com )
// 
// 12/Jan/2018
//
//
///////////////////////////////////////////////////////////////

#include "Utility/GeometricPrimitives.h"
#include "Utility/HelpDefine.h"
#include "Polygon.h"
#include <igl/triangle/triangulate.h>
#include <iostream>
#include <set>
//**************************************************************************************//
//                                    Initialization
//**************************************************************************************//

template <typename Scalar>
_Polygon<Scalar>::_Polygon()
{
	polyType = POLY_NONE;
}

template <typename Scalar>
_Polygon<Scalar>::~_Polygon()
{
    clear();
}

template <typename Scalar>
_Polygon<Scalar>::_Polygon(const _Polygon<Scalar> &poly)
{
    vers.clear();
    for(pVertex vertex: poly.vers){
        vers.push_back(make_shared<VPoint<Scalar>>(*vertex));
    }

    texs.clear();
    for(pVTex tex: poly.texs){
        texs.push_back(make_shared<VTex<Scalar>>(*tex));
    }

    edge_at_boundary.clear();
    for(bool vb: poly.edge_at_boundary){
        edge_at_boundary.push_back(vb);
    }

    this->polyType = poly.polyType;
}

//**************************************************************************************//
//                                    modify operation
//**************************************************************************************//

// 1) will affect computed data
template <typename Scalar>
void _Polygon<Scalar>::setVertices(vector<Vector3> _vers)
{

    vers.clear();
    for (size_t i = 0; i < _vers.size(); i++)
    {
        pVertex vertex = make_shared<VPoint<Scalar>>(_vers[i]);
        vers.push_back(vertex);
    }
    edge_at_boundary.clear();
    edge_at_boundary.resize(_vers.size(), true);
}

template <typename Scalar>
void _Polygon<Scalar>::setVertices2(vector<Vector2> _vers){
    vers.clear();
    for (size_t i = 0; i < _vers.size(); i++)
    {
        pVertex vertex = make_shared<VPoint<Scalar>>(Vector3(_vers[i].x(), _vers[i].y(), 0));
        vers.push_back(vertex);
    }
    edge_at_boundary.clear();
    edge_at_boundary.resize(_vers.size(), true);
}

template <typename Scalar>
size_t _Polygon<Scalar>::push_back(VPoint<Scalar> pt)
{
    pVertex vertex = make_shared<VPoint<Scalar>>(pt);
    vers.push_back(vertex);
    return vers.size();
}

template <typename Scalar>
size_t _Polygon<Scalar>::push_back(Vector3 pt, Vector2 tex)
{
    pVertex vertex = make_shared<VPoint<Scalar>>(pt);
    vers.push_back(vertex);

    pVTex ptex = make_shared<VTex<Scalar>>(tex);
    texs.push_back(ptex);

    return vers.size();
}

// 2) will not affect computed data

template <typename Scalar>
void _Polygon<Scalar>::reverseVertices()
{
    // Reverse vertices
    vector<pVertex> newVers;
    for (int i = (int)(vers.size()) - 1; i >= 0; i--)
    {
        pVertex vertex = make_shared<VPoint<Scalar>>(*vers[i]);
        newVers.push_back(vertex);
    }

    vers = newVers;
}

template <typename Scalar>
void _Polygon<Scalar>::translatePolygon(Vector3 transVec)
{
    for (size_t i = 0; i < vers.size(); i++)
    {
        vers[i]->pos += transVec;
    }
}

template <typename Scalar>
void _Polygon<Scalar>::translatePolygonTex(Vector2 transVec)
{
    for (size_t i = 0; i < texs.size(); i++)
    {
        texs[i]->texCoord += transVec;
    }
}

/***********************************************
 *                                             *
 *             read only      operation        *
 *                                             *
 ***********************************************/

template <typename Scalar>
bool _Polygon<Scalar>::checkEquality(const _Polygon &poly) const
{
	if (this->size() != poly.size())
		return false;

	const _Polygon<Scalar> &A = *this;
	const _Polygon<Scalar> &B = poly;

	int id;
    for(id = 0; id < A.size(); id++)
    {
        if((A.vers[id]->pos - B.vers[0]->pos).norm() < FLOAT_ERROR_SMALL){
            break;
        }
    }
    if(id == A.size()) return false;

    for(size_t jd = 0; jd < A.size(); jd++)
    {
        int Aij = (id + jd) % A.size();
        if((A.vers[Aij]->pos - B.vers[jd]->pos).norm() > FLOAT_ERROR_SMALL){
            return false;
        }
    }

    return true;
}

template <typename Scalar>
void _Polygon<Scalar>::print() const
{
	printf("verNum: %lu \n", vers.size());
	for (size_t i = 0; i < vers.size(); i++)
	{
		printf("(%6.3f, %6.3f, %6.3f) \n", vers[i]->pos.x(), vers[i]->pos.y(), vers[i]->pos.z());
	}
	printf("\n");
}
template <typename Scalar>
Box<Scalar> _Polygon<Scalar>::bbox() const{
    vector<Vector3> points = getVertices();
    Box<Scalar> bbx(points);
    return bbx;
}

template <typename Scalar>
Matrix<Scalar, 3, 1> _Polygon<Scalar>::center() const
{
    Vector3 _center = Vector3(0, 0, 0);
	for (size_t i = 0; i < vers.size(); i++)
	{
        _center += vers[i]->pos;
	}
    _center = _center / vers.size();
	return _center;
}

template <typename Scalar>
Matrix<Scalar, 3, 1> _Polygon<Scalar>::centroid() const{

    shared_ptr<_Polygon<Scalar>> proj_polygon = make_shared<_Polygon<Scalar>>();

    Vector3 xaxis, yaxis, origin;
    computeFrame(xaxis, yaxis, origin);
    proj_polygon = projectPolygonInto2D(xaxis, yaxis, origin);

    vector<Line<Scalar>> lines;
    lines = proj_polygon->convertToLines();

    Scalar volume = 0.0;
    //volume
    for(int id = 0; id < lines.size(); id++){
        Scalar xi = lines[id].point1.x();
        Scalar xi_p1 = lines[id].point2.x();
        Scalar yi = lines[id].point1.y();
        Scalar yi_p1 = lines[id].point2.y();
        volume += 1.0 / 2.0 * (xi * yi_p1 - xi_p1 * yi);
    }

    //com
    Matrix<Scalar, 3, 1> center_of_mass = Matrix<Scalar, 3, 1>(0, 0, 0);
    for(int id = 0; id < lines.size(); id++){
        Scalar xi = lines[id].point1.x();
        Scalar xi_p1 = lines[id].point2.x();
        Scalar yi = lines[id].point1.y();
        Scalar yi_p1 = lines[id].point2.y();
        center_of_mass.x() += (xi + xi_p1) * (xi * yi_p1 - xi_p1 * yi);
        center_of_mass.y() += (yi + yi_p1) * (xi * yi_p1 - xi_p1 * yi);
    }

    if(std::abs(volume) > FLOAT_ERROR_SMALL){
        center_of_mass = center_of_mass / (6 * volume);
    }
    else{
        std::cout << "Volume Compute Error" << std::endl;
    }

    return center_of_mass[0] * xaxis + center_of_mass[1] * yaxis + origin;
}


template <typename Scalar>
Matrix<Scalar, 3, 1> _Polygon<Scalar>::normal() const
{
	Vector3 _center = center();
	Vector3 tempNor(0, 0, 0);
	for(int id = 0; id < (int)(vers.size()) - 1; id++)
	{
	    tempNor += (vers[id]->pos - _center).cross(vers[id + 1]->pos - _center);
	}

	if(tempNor.norm() < FLOAT_ERROR_LARGE)
	    return Vector3(0, 0, 0);

	// normal already normalized
	Vector3 _normal = computeFitedPlaneNormal();
	if(_normal.dot(tempNor) < 0) _normal *= -1;
	return _normal;
}

// see https://www.ilikebigbits.com/2015_03_04_plane_from_points.html
template <typename Scalar>
Matrix<Scalar, 3, 1> _Polygon<Scalar>::computeFitedPlaneNormal() const
{
	if(vers.size() < 3)
	{
		return Vector3(0, 0, 0);
	}

	Vector3 _center = center();

	double xx, xy, xz, yy, yz, zz;
	xx = xy = xz = yy = yz = zz = 0;
	for(pVertex ver: vers)
	{
		Vector3 r = ver->pos - _center;
		xx += r.x() * r.x();
		xy += r.x() * r.y();
		xz += r.x() * r.z();
		yy += r.y() * r.y();
		yz += r.y() * r.z();
		zz += r.z() * r.z();
	}

	double det_x = yy*zz - yz*yz;
	double det_y = xx*zz - xz*xz;
	double det_z = xx*yy - xy*xy;

	double maxDet = std::max(det_x, std::max(det_y, det_z));
	if(maxDet <= 0){
		return Vector3(0, 0, 0);
	}

	Vector3 _normal;
	if(maxDet == det_x)
	{
        _normal = Vector3(det_x, xz*yz - xy*zz, xy*yz - xz*yy);
	}
	else if(maxDet == det_y)
	{
        _normal = Vector3(xz*yz - xy*zz, det_y, xy*xz - yz*xx);
	}
	else {
        _normal = Vector3(xy*yz - xz*yy, xy*xz - yz*xx, det_z);
	}

	return _normal.normalized();
}

template <typename Scalar>
Scalar _Polygon<Scalar>::signed_area() const{
    Scalar signedArea = 0;
    for (size_t i = 0; i < vers.size(); i++)
    {
        Vector3 currVer = vers[i]->pos;
        Vector3 nextVer = vers[(i + 1) % vers.size()]->pos;
        signedArea += 0.5 * (currVer.cross(nextVer)).z();
    }
    return signedArea;
}

template <typename Scalar>
Scalar _Polygon<Scalar>::area() const
{
    Vector3 _normal = normal();

    Scalar signedArea = 0;
    for (size_t i = 0; i < vers.size(); i++)
    {
        Vector3 currVer = vers[i]->pos;
        Vector3 nextVer = vers[(i + 1) % vers.size()]->pos;
        signedArea += 0.5 * (_normal.dot(currVer.cross(nextVer)));
    }

	return std::abs(signedArea);
}

template <typename Scalar>
Scalar _Polygon<Scalar>::average_edge() const
{
	Scalar avgEdgeLen = 0;

	for (size_t i = 0; i < vers.size(); i++)
	{
		Vector3 currVer = vers[i]->pos;
		Vector3 nextVer = vers[(i + 1) % vers.size()]->pos;
        avgEdgeLen += (nextVer - currVer).norm();
	}

	avgEdgeLen /= vers.size();

	return avgEdgeLen;
}

template <typename Scalar>
Scalar _Polygon<Scalar>::max_radius() const
{
    Scalar MaxRadius = 0;

    Vector3 origin = center();
    for (size_t i = 0; i < vers.size(); i++)
    {
        MaxRadius = std::max((vers[i]->pos - origin).norm(), MaxRadius);
    }
    return MaxRadius;
}

template <typename Scalar>
void _Polygon<Scalar>::computeFrame(Vector3 &x_axis, Vector3 &y_axis, Vector3 &origin) const
{
	Vector3 _normal = normal();
    origin = center();

	x_axis = _normal.cross(Vector3(1, 0, 0));
	if(x_axis.norm() < FLOAT_ERROR_LARGE)
	    x_axis = _normal.cross(Vector3(0, 1, 0));
	x_axis.normalize();

	y_axis = _normal.cross(x_axis);
	y_axis.normalize();
}

template <typename Scalar>
shared_ptr<_Polygon<Scalar>> _Polygon<Scalar>::projectPolygonInto2D(Vector3 &x_axis, Vector3 &y_axis, Vector3 &origin) const{
    shared_ptr<_Polygon<Scalar>> proj_polygon = make_shared<_Polygon<Scalar>>();
    for(int id = 0; id < vers.size(); id++){
        Vector3 pt = pos(id);
        Scalar u = (pt - origin).dot(x_axis);
        Scalar v = (pt - origin).dot(y_axis);
        proj_polygon->push_back(Vector3(u, v, 0));
    }
    return proj_polygon;
}

// https://www.mn.uio.no/math/english/people/aca/michaelf/papers/wach_mv.pdf
template <typename Scalar>
vector<Scalar> _Polygon<Scalar>::computeBaryCentric(Vector2 pt) const{
    vector<Scalar> barycentric;

    if(texs.empty()){
        return barycentric;
    }

    vector<Scalar> wi;
    Scalar sumw = 0;
    for(int id = 0; id < texs.size(); id++)
    {
        Vector2 mv = texs[(id - 1 + texs.size()) % texs.size()]->texCoord;
        Vector2 v =  texs[id]->texCoord;
        Vector2 pv = texs[(id + 1 + texs.size()) % texs.size()]->texCoord;

        Triangle<Scalar> A_mv_v_pv(mv, v, pv);
        Triangle<Scalar> A_pt_mv_v(pt, mv, v);
        Triangle<Scalar> A_pt_v_pv(pt, v, pv);

        Scalar area_pt_mv_v = A_pt_mv_v.computeSignedArea();
        Scalar area_pt_v_pv = A_pt_v_pv.computeSignedArea();

        // line case 1)
        if(std::abs(area_pt_mv_v) < FLOAT_ERROR_SMALL){
            vector<Scalar> line = computeBaryCentric(mv, v, pt);
            barycentric.resize(texs.size(), 0);
            barycentric[(id - 1 + texs.size()) % texs.size()] = line[0];
            barycentric[id] = line[1];
            return barycentric;
        }

        // line case 2)
        if(std::abs(area_pt_v_pv) < FLOAT_ERROR_SMALL){
            vector<Scalar> line = computeBaryCentric(v, pv, pt);
            barycentric.resize(texs.size(), 0);
            barycentric[id] = line[0];
            barycentric[(id + 1) % texs.size()] = line[1];
            return barycentric;
        }

        Scalar w = A_mv_v_pv.computeSignedArea() / area_pt_mv_v / area_pt_v_pv;
        wi.push_back(w);
        sumw += w;
    }

    for(size_t id = 0; id < texs.size(); id++){
        barycentric.push_back(wi[id] / sumw);
    }

    return barycentric;
}

template <typename Scalar>
vector<Scalar> _Polygon<Scalar>::computeBaryCentric(Vector2 sta, Vector2 end, Vector2 pt) const{

    // notice we don't handle the case where pt is not on the segment (sta, end)
    if((pt - sta).norm() < FLOAT_ERROR_SMALL){
        return vector<Scalar>({1, 0});
    }

    if((pt - end).norm() < FLOAT_ERROR_SMALL){
        return vector<Scalar>({0, 1});
    }

    if((sta - end).norm() < FLOAT_ERROR_SMALL){
        return vector<Scalar>();
    }

    Scalar l0 = (pt - end).norm() / (sta - end).norm();
    return vector<Scalar>({l0, 1 - l0});
}



template <typename Scalar>
void _Polygon<Scalar>::triangulateNaive(vector<pTriangle> &triList) const
{
    triList.clear();

    if(vers.size() < 3) return;

    if(vers.size() == 3)
    {
        pTriangle tri = make_shared<Triangle<Scalar>>();
        for(int kd = 0; kd < 3; kd++){
            tri->v[kd] = vers[kd]->pos;
        }
        triList.push_back(tri);
    }
    else{
        Vector3 _center = center();
        for (size_t i = 0; i < vers.size(); i++)
        {
            pTriangle tri = make_shared<Triangle<Scalar>>();
            tri->v[0] = vers[i]->pos;
            tri->v[1] = vers[(i + 1) % vers.size()]->pos;
            tri->v[2] = _center;
            triList.push_back(tri);
        }
    }

    //assign norm
    Vector3 nrm = normal();
    for(auto tri: triList){
        for(int kd = 0; kd < 3; kd++){
            tri->vf[kd] = nrm;
        }
    }

    return;
}

template <typename Scalar>
int _Polygon<Scalar>::getPtVerID(_Polygon<Scalar>::Vector3 point) const
{
	for (size_t i = 0; i < vers.size(); i++)
	{
		Scalar dist = (point - vers[i]->pos).norm();

		// Note: this threshold depends on the scale of elements
		if (dist < FLOAT_ERROR_LARGE)
		{
			return i;
		}
	}

	return ELEMENT_OUT_LIST;
}
template <typename Scalar>
void _Polygon<Scalar>::triangulate(vector<pTriangle> &triList,
                                   std::string settings,
                                   bool input_triangle) const
{

    if(area() < FLOAT_ERROR_SMALL){
        return;
    }

    triList.clear();

    if(vers.size() < 3) return;

    if(vers.size() == 3 && !input_triangle)
    {
        pTriangle tri = make_shared<Triangle<Scalar>>();
        for(int kd = 0; kd < 3; kd++) {
            tri->v[kd] = vers[kd]->pos;
        }
        triList.push_back(tri);
    }
    else{
        Vector3 x_axis, y_axis, origin;
        computeFrame(x_axis, y_axis, origin);

        Eigen::Matrix<Scalar, Eigen::Dynamic, 2> V(vers.size(), 2), V2;
        Eigen::Matrix<int, Eigen::Dynamic, 2> E(vers.size(), 2);
        Eigen::Matrix<int, Eigen::Dynamic, 3> F;
        Eigen::Matrix<Scalar, 0, 2> H;

        for(int id = 0; id < vers.size(); id++){
            Scalar x = (vers[id]->pos - origin).dot(x_axis);
            Scalar y = (vers[id]->pos - origin).dot(y_axis);
            V(id, 0) = x;
            V(id, 1) = y;
            E(id, 0) = id;
            E(id, 1) = (id + 1) % size();
        }

        igl::triangle::triangulate(V, E, H, settings, V2, F);
        Eigen::Matrix<Scalar, Eigen::Dynamic, 3> V2_3d(V2.rows(), 3);
        for(int id = 0; id < V2.rows(); id++){
            V2_3d.row(id) = V2(id, 0) * x_axis + V2(id , 1) * y_axis + origin;
        }

        for(int id = 0; id < F.rows(); id++){
            shared_ptr<Triangle<Scalar>> tri = make_shared<Triangle<Scalar>>();
            for(int kd = 0; kd < 3; kd++)
            {
                tri->v[kd] = V2_3d.row(F(id, kd));
            }
            triList.push_back(tri);
        }
    }

    //assign norm
    Vector3 nrm = normal();
    for(auto tri: triList){
        for(int kd = 0; kd < 3; kd++){
            tri->vf[kd] = nrm;
        }
    }
    return;
}

template<typename Scalar>
void _Polygon<Scalar>::removeDuplicatedVertices(double eps) {
    vector<shared_ptr<VPoint<Scalar>>> new_vers;
    vector<shared_ptr<VTex<Scalar>>> new_texs;
    vector<bool> new_boundary;

    if(!vers.empty()) new_vers.push_back(vers[0]);
    if(!texs.empty()) new_texs.push_back(texs[0]);
    if(!edge_at_boundary.empty()) new_boundary.push_back(edge_at_boundary[0]);

    for(int id = 1; id < size(); id++)
    {

        if((pos(id) - pos(id - 1)).norm() < eps){
            continue;
        }
        if(id == (int) size() - 1 && (pos(id) - pos(id + 1)).norm() < eps){
            continue;
        }
        new_vers.push_back(vers[id]);
        if(texs.size() > id)             new_texs.push_back(texs[id]);
        if(edge_at_boundary.size() > id) new_boundary.push_back(edge_at_boundary[id]);
    }

    vers = new_vers;
    texs = new_texs;
    edge_at_boundary = new_boundary;
}

template<typename Scalar>
vector<Line<Scalar>> _Polygon<Scalar>::convertToLines() const{
    vector<Line<Scalar>> lines;
    for(int id = 0; id < size(); id++){
        lines.push_back(Line<Scalar>(pos(id), pos(id + 1)));
    }
    return lines;
}

template<typename Scalar>
bool _Polygon<Scalar>::checkInsidePolygon(Vector3 pt){
    vector<Vector3> vi, ei;
    int N = size();

    //compute vi, ei;
    for( int i=0 ; i < N; i++) {
        Vector3 e = pos(i + 1) - pos(i);
        Vector3 v =  pt - pos(i);
        vi.push_back(v);
        ei.push_back(e);
    }

    //compute winding number
    int wn =0;
    for( int i = 0; i < N; i++) {
        bool cond1 = 0. <= vi[i].y();
        bool cond2 = 0. > vi[(i + 1) % N].y();
        Scalar val3= ei[i].cross(vi[i]).z(); //isLeft
        wn+= cond1 && cond2 && val3> 0. ? 1 : 0; // have  a valid up intersect
        wn-= !cond1 && !cond2 && val3<0. ? 1 : 0; // have  a valid down intersect
    }

    Scalar d = std::numeric_limits<double>::max();
    //compute distance
    for( int i=0; i < N; i++)
    {
        Vector3 pq = vi[i] - ei[i] * std::clamp( (double) (vi[i].dot(ei[i]))/(ei[i].dot(ei[i])), 0.0, 1.0 );
        d = std::min((double)d, (double)sqrt(pq.dot(pq)));
    }

    if(wn == 0){
        return false;
    }
    else{
        return true;
    }
}

template<typename Scalar>
void _Polygon<Scalar>::triangulateQuad(int num_x, int num_y, vector<shared_ptr<Triangle<Scalar>>> &tris)
{

    if(size() != 4)
        return;

    Vector3 origin = center();
    Vector3 xaxis = pos(1) - pos(0); xaxis.normalize();
    Vector3 yaxis = pos(3) - pos(0); yaxis.normalize();

    shared_ptr<_Polygon<Scalar>> projPolygon = projectPolygonInto2D(xaxis, yaxis, origin);

    Box<Scalar> box = projPolygon->bbox();

    int DX[4] = {0, 1, 1, 0};
    int DY[4] = {0, 0, 1, 1};
    int TRI[2][3] = {{0, 1, 2}, {2, 3, 0}};
    //sample space
    for(int id = 0; id < num_x; id++)
    {
        for(int jd = 0; jd < num_y; jd++)
        {
            Vector3 pt = Vector3(box.minPt.x() + box.size.x() / num_x * id,
                                 box.minPt.y() + box.size.y() / num_y * jd,
                                 0);
            vector<Vector3> proj_pts;
            for(int kd = 0; kd < 4; kd++)
            {
                Vector3 new_pt = pt;
                new_pt.x() += DX[kd] * box.size.x() / num_x;
                new_pt.y() += DY[kd] * box.size.y() / num_y;
                proj_pts.push_back(new_pt);
            }

            for(int kd = 0; kd < 2; kd++)
            {
                shared_ptr<Triangle<Scalar>> tri = make_shared<Triangle<Scalar>>();
                for(int ld = 0; ld < 3; ld++)
                {
                    Vector3 proj_pt = proj_pts[TRI[kd][ld]];
                    Vector3 origin_pt = proj_pt[0] *xaxis + proj_pt[1] * yaxis + origin;
                    tri->v[ld] = origin_pt;
                }
                tris.push_back(tri);
            }
        }
    }

    return;
}



template class _Polygon<double>;
template class _Polygon<float>;