///////////////////////////////////////////////////////////////
//
// Mesh.h
//
//   Polygonal Mesh
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 09/Jan/2018
//
//
///////////////////////////////////////////////////////////////


#ifndef _POLY_MESH_H
#define _POLY_MESH_H

#include "TopoLite/Utility/GeometricPrimitives.h"
#include "MeshBase.h"
#include "TopoLite/Utility/PolyPolyBoolean.h"
#include "Polygon.h"
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

using Eigen::Matrix;

template<typename Scalar>
class PolyMesh : public MeshBase<Scalar>
{
public:

    using MeshBase<Scalar>::getVarList;

    using pPolygon = shared_ptr<_Polygon<Scalar>> ;

    using wpPolygon = weak_ptr<_Polygon<Scalar>> ;

    using pTriangle = shared_ptr<Triangle<Scalar>> ;

    typedef Matrix<Scalar, 3, 1> Vector3;

    typedef Matrix<Scalar, 2, 1> Vector2;

    typedef Matrix<int, 3, 1> Vector3i;

    typedef shared_ptr<VPoint<Scalar>> pVertex;

    typedef shared_ptr<VTex<Scalar>> pVTex;

    typedef Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;

    typedef Matrix<int, Eigen::Dynamic, Eigen::Dynamic> MatrixXi;

public:

    struct sort_vertex
    {
        Vector3 pos;
        int vID;
        double eps;
        shared_ptr<VPoint<Scalar>> ptr;
    };

    struct sort_vertex_compare
    {
        bool operator()(const sort_vertex& A, const sort_vertex& B) const
        {
            double eps = A.eps / 2;

            if (A.pos[0] - B.pos[0] < -eps)
                return true;
            if (A.pos[0] - B.pos[0] > eps)
                return false;

            if (A.pos[1] - B.pos[1] < -eps)
                return true;
            if (A.pos[1] - B.pos[1] > eps)
                return false;

            if (A.pos[2] - B.pos[2] < -eps)
                return true;
            if (A.pos[2] - B.pos[2] > eps)
                return false;

            return false;
        }
    };

public:

    //Storage
	vector<pPolygon> polyList;           // Faces of polygonal mesh

    bool texturedModel;                  // Whether this mesh has texture

public:

	//Computed

    vector<pVertex> vertexList;          // Vertex Position of polygonal mesh

    vector<pVTex> textureList;               // Texture Coordinate

public:
    
    PolyMesh(std::shared_ptr<InputVarList> var) : MeshBase<Scalar>::MeshBase(var){
        clear();
    }

    PolyMesh(const PolyMesh &_mesh);

	~PolyMesh();


/***********************************************
 *             modify mesh operation           *
 ***********************************************/

public:

    void clear();

    void update(){
        removeDuplicatedVertices();
        computeTextureList();
    }

    std::pair<Matrix<Scalar, 3, 1>, Scalar> normalize();

    void fromEigenMesh(const MatrixX &V, const MatrixX &T, const MatrixXi &F);

    void fromTriLists(vector<shared_ptr<Triangle<Scalar>>> triLists);

    void setPolyLists(vector<pPolygon> _polyList);

    void flipFace();

    /*!
     * \brief: parse from json file
     * \param mesh_json: json object of the mesh
     * \return: whether read the json object successfully
     */

    bool parse(const nlohmann::json &mesh_json);

    // Read OBJ File
    /*!
     * \brief: Read OBJ File (from file)
     * \param fileName: filename of an input file
     * \param normalized: scale the mode into [-1, 1] x [-1, 1] x [-1, 1] box;
     */
    bool readOBJModel(const char *fileName, bool normalized);
    
    /*!
     * \brief: Read OBJ File (from matrix)
     * \param V: vertice's position arrary
     * \param TC: vertex texture coordinates array
     * \param F: face index (tri or polygon)
     * \param FTC: texture face index (tri or polygon)
     * \param normalized: scale the mode into [-1, 1] x [-1, 1] x [-1, 1] box;
     */
    bool readOBJModel(const vector<vector<double>> &V, const vector<vector<double>> &TC, const vector<vector<int>> &F, const vector<vector<int>> &FTC, bool normalized);

    void removeDuplicatedVertices(double eps = FLOAT_ERROR_LARGE);

    void mergeFaces(double eps = 1e-3);

    void computeBoundaryLoops(vector<shared_ptr<_Polygon<Scalar>>> &poly);

    // Update vertexList and textureList

    void computeVertexList();

    void computeTextureList();

    // Transform Mesh
    void translateMesh(Vector3 move);

    void scaleMesh(Vector3 scale);

    void rotateMesh(Vector3 rotCenter, Vector3 rotAxis, Scalar rotAngle);

public:
/***********************************************
 *             read only mesh operation        *
 ***********************************************/

	void print() const;

    vector<Vector3> getVertices() const{
        vector<Vector3> pointLists;
        for(pVertex vertex: vertexList){
            pointLists.push_back(vertex->pos);
        }
        return pointLists;
    }

    shared_ptr<PolyMesh<Scalar>> getTextureMesh() const;

    //Save OBJ File
    void writeOBJModel(const char *objFileName, bool triangulate = false) const;

    nlohmann::json dump() const;

    //convert to triangle mesh
    void convertToTriLists(vector<pTriangle> &triList, bool compute_normal = false, bool convex = true) const override;

    //convert to Eigen mesh
    void convertPosToEigenMesh(MatrixX &V, MatrixXi &F, Eigen::VectorXi &C);

    void convertTexToEigenMesh(MatrixX &V, MatrixXi &F, Eigen::VectorXi &C);

    void convertPosTexToEigenMesh(MatrixX &V, MatrixX &T, MatrixXi &F);

    // Auxiliary Data Computation
    Box<Scalar> bbox() const;

    Box<Scalar> texBBox() const;

    Vector3 centroid() const;

    Vector3 center() const override;

    Scalar volume() const;

    Vector3 lowestPt() const;

    size_t size() const {return polyList.size();}

    size_t n_vertices() const override {return vertexList.size();}

    vector<Line<Scalar>> getWireFrame() const override;

private:

    Scalar computeVolume(vector<pTriangle> triList) const;

    Vector3 computeCentroid(vector<pTriangle> triList) const;

    Vector3 computeExtremeVertex(Vector3 rayDir) const;
};
#endif
