//
// Created by ziqwang on 13.01.21.
//

#ifndef TOPOLITE_TRIMESHWITHFEATURES_H
#define TOPOLITE_TRIMESHWITHFEATURES_H

#include "PolyMesh.h"
#include "TriMesh.h"
#include "Polygon.h"
#include <fstream>
#include "igl/boundary_loop.h"
#include "igl/readOBJ.h"
#include <filesystem>

template <typename Scalar>
class TriMeshWithFeatures : public TopoObject{

public:

    TriMeshWithFeatures(shared_ptr<InputVarList> varList):
    TopoObject(varList){

    }

public:

    vector<shared_ptr<TriMesh<Scalar>>> feature_meshes;

    shared_ptr<PolyMesh<Scalar>> contact_mesh;

public:

    void readFromFile(std::string filename,
                      vector<shared_ptr<TriMeshWithFeatures<Scalar>>> &feasture_meshes);

    void readFromFilePurePlanarBlocks(std::string filename,
                                      vector<shared_ptr<TriMeshWithFeatures<Scalar>>> &feasture_meshes,
                                      vector<bool> &boundary);

    void readFromListofFiles(std::string foldername,
                             std::string objname,
                             int number_of_parts,
                             vector<shared_ptr<TriMeshWithFeatures<Scalar>>> &feasture_meshes);
};

template<typename Scalar>
void TriMeshWithFeatures<Scalar>::readFromFile(std::string filename,
                                               vector<shared_ptr<TriMeshWithFeatures<Scalar>>> &feasture_meshes)
{
    feature_meshes.clear();
    contact_mesh.reset();

    FILE *fin;
    fin = fopen(filename.c_str(), "r");

    int num_of_part = 0;
    fscanf(fin, "%d", &num_of_part);

    for(int partID = 0; partID < num_of_part; partID++)
    {

        //planar contact
        int num_of_planar_contact_curve = 0;
        fscanf(fin, "%d", &num_of_planar_contact_curve);

        vector<shared_ptr<_Polygon<Scalar>>> part_polygons;
        for(int pID = 0; pID < num_of_planar_contact_curve; pID++)
        {
            int num_of_pts = 0;
            fscanf(fin, "%d", &num_of_pts);
            shared_ptr<_Polygon<double>> poly;
            poly = make_shared<_Polygon<double>>();
            for(int vID = 0; vID < num_of_pts; vID++){
                double x, y, z;
                fscanf(fin, "%lf %lf %lf", &x, &y, &z);
                Vector3d pt(x, y, z);
                poly->push_back(pt);
            }
            part_polygons.push_back(poly);
        }

        //curved contact
        int num_of_curved_contact = 0;
        fscanf(fin, "%d", &num_of_curved_contact);
        vector<shared_ptr<TriMesh<Scalar>>> part_meshes;
        for(int fID = 0; fID < num_of_curved_contact; fID++)
        {
            int num_of_triangles;
            fscanf(fin, "%d", &num_of_triangles);
            vector<shared_ptr<Triangle<double>>> triangles;
            for(int it = 0; it < num_of_triangles; it++)
            {
                shared_ptr<Triangle<double>> tri = make_shared<Triangle<double>>();
                for(int kd = 0; kd < 3; kd++){
                    double x, y, z;
                    fscanf(fin, "%lf %lf %lf", &x, &y, &z);
                    Vector3d pt(x, y, z);
                    tri->v[kd] = pt;
                }
                triangles.push_back(tri);
            }
            shared_ptr<TriMesh<double>> triMesh = make_shared<TriMesh<double>>(triangles, getVarList());
            triMesh->recomputeNormal();
            part_meshes.push_back(triMesh);
        }

        shared_ptr<TriMeshWithFeatures> part_mesh_features
        = make_shared<TriMeshWithFeatures>(getVarList());

        part_mesh_features->feature_meshes = part_meshes;
        part_mesh_features->contact_mesh = make_shared<PolyMesh<double>>(getVarList());
        part_mesh_features->contact_mesh->setPolyLists(part_polygons);
        feasture_meshes.push_back(part_mesh_features);
    }
}

template<typename Scalar>
void TriMeshWithFeatures<Scalar>::readFromFilePurePlanarBlocks(std::string filename,
                                  vector<shared_ptr<TriMeshWithFeatures<Scalar>>> &feasture_meshes,
                                  vector<bool> &boundary) {
    feature_meshes.clear();
    contact_mesh.reset();

    FILE *fin;
    fin = fopen(filename.c_str(), "r");

    int num_of_part = 0;
    fscanf(fin, "%d", &num_of_part);

    for (int partID = 0; partID < num_of_part; partID++) {

        //planar contact
        int number_of_face = 0;
        int atBoundary = 0;
        fscanf(fin, "%d %d", &number_of_face, &atBoundary);
        boundary.push_back(atBoundary);

        vector<shared_ptr<_Polygon<Scalar>>> part_polygons;
        for (int pID = 0; pID < number_of_face; pID++) {
            int num_of_pts = 0;
            fscanf(fin, "%d", &num_of_pts);
            shared_ptr<_Polygon<double>> poly;
            poly = make_shared<_Polygon<double>>();
            for (int vID = 0; vID < num_of_pts; vID++) {
                double x, y, z;
                fscanf(fin, "%lf %lf %lf", &x, &y, &z);
                Vector3d pt(x, y, z);
                poly->push_back(pt);
            }
            part_polygons.push_back(poly);
        }
        shared_ptr<TriMeshWithFeatures> part_mesh_features
                = make_shared<TriMeshWithFeatures>(getVarList());

        part_mesh_features->contact_mesh = make_shared<PolyMesh<double>>(getVarList());
        part_mesh_features->contact_mesh->setPolyLists(part_polygons);
        feasture_meshes.push_back(part_mesh_features);
    }
}

template<typename Scalar>
void TriMeshWithFeatures<Scalar>::readFromListofFiles(std::string foldername,
                                                      std::string objname,
                                                      int number_of_parts,
                                                      vector<shared_ptr<TriMeshWithFeatures<Scalar>>> &feasture_meshes){

    vector<Vector3d> pts;
    vector<Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>> Vs;
    vector<Matrix<int, Eigen::Dynamic, Eigen::Dynamic>> Fs;

    for(int id = 0; id < number_of_parts; id++)
    {
        std::string filename = foldername + "/" + objname + to_string(id) + ".obj";
        Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> V;
        Matrix<int, Eigen::Dynamic, Eigen::Dynamic> F;
        igl::readOBJ(filename, V, F);
        Vs.push_back(V);
        Fs.push_back(F);

        shared_ptr<TriMeshWithFeatures<Scalar>> feature_mesh = make_shared<TriMeshWithFeatures<Scalar>>(getVarList());
        feature_mesh->contact_mesh = make_shared<PolyMesh<Scalar>>(getVarList());
        feasture_meshes.push_back(feature_mesh);

        for(int kd = 0; kd < V.rows(); kd ++){
            pts.push_back(V.row(kd));
        }
    }


    vector<Plane<Scalar>> cut_meshes;
    {
        std::string filename = foldername + "/" + objname + "_data.txt";

        FILE *fin;
        fin = fopen(filename.c_str(), "r");
        if(std::filesystem::exists(filename))
        {
            //planar contact
            int number_of_planes = 0;
            fscanf(fin, "%d", &number_of_planes);

            for(int pID = 0; pID < number_of_planes; pID++)
            {
                Matrix<Scalar, 3, 1> normal, center;
                double x, y, z;
                //center
                fscanf(fin, "%lf %lf %lf", &x, &y, &z);
                center = Matrix<Scalar, 3, 1>(x, z, -y);

                //normal
                fscanf(fin, "%lf %lf %lf", &x, &y, &z);
                normal = Matrix<Scalar, 3, 1>(x, z, -y);

                //plane
                Plane<Scalar> plane;
                plane.point = center;
                plane.normal = normal;
                cut_meshes.push_back(plane);
            }
        }
        fclose(fin);
    }


    vector<vector<shared_ptr<Triangle<Scalar>>>> non_contact_meshes;
    vector<shared_ptr<PolyMesh<Scalar>>> contact_meshes;

    contact_meshes.resize(Vs.size());
    non_contact_meshes.resize(Vs.size());

    for(int id = 0; id < Vs.size(); id++)
    {
        contact_meshes[id] = make_shared<PolyMesh<Scalar>>(getVarList());

        vector<vector<Eigen::RowVector3i>> face_which_planes;
        face_which_planes.resize(cut_meshes.size());
        for(int fID = 0; fID < Fs[id].rows(); fID++)
        {
            bool is_belong_to_contact = false;
            for(int kd = 0; kd < cut_meshes.size(); kd++)
            {
                bool is_on_cut_mesh = true;
                for(int jd = 0; jd < 3; jd++)
                {
                    Eigen::Matrix<Scalar, 3, 1> pt = Vs[id].row(Fs[id](fID, jd));
                    Scalar distance = (pt - cut_meshes[kd].point).dot(cut_meshes[kd].normal);
                    if(std::abs(distance) > 1e-2){
                        is_on_cut_mesh = false;
                        break;
                    }
                }

                if(is_on_cut_mesh){
                    face_which_planes[kd].push_back(Fs[id].row(fID));
                    is_belong_to_contact = true;
                    break;
                }
            }

            if(!is_belong_to_contact)
            {
                shared_ptr<Triangle<Scalar>> tri = make_shared<Triangle<Scalar>>();
                tri->v[0] = Vs[id].row(Fs[id](fID, 0));
                tri->v[1] = Vs[id].row(Fs[id](fID, 1));
                tri->v[2] = Vs[id].row(Fs[id](fID, 2));
                non_contact_meshes[id].push_back(tri);
            }
        }

        for(int cID = 0; cID < cut_meshes.size(); cID++)
        {
            if(!face_which_planes[cID].empty())
            {
                vector<vector<int>> Ls;

                Eigen::Matrix<int, Eigen::Dynamic, 3> F(face_which_planes[cID].size(), 3);
                for(int id = 0; id < face_which_planes[cID].size(); id++){
                    F.row(id) = face_which_planes[cID][id];
                }

                igl::boundary_loop(F, Ls);

                for(const vector<int> &L: Ls)
                {
                    shared_ptr<_Polygon<Scalar>> contact_polygon = make_shared<_Polygon<Scalar>>();
                    for(int kd = 0; kd < L.size(); kd++)
                    {
                        Matrix<Scalar, 3, 1> pt = Vs[id].row(L[kd]);
                        contact_polygon->push_back(pt);
                    }

                    contact_meshes[id]->polyList.push_back(contact_polygon);
                }

            }
        }
    }


    for(int id = 0; id < feasture_meshes.size(); id++){
        //feasture_meshes[id]->contact_mesh = contact_meshes[id];
        shared_ptr<TriMesh<Scalar>> triMesh = make_shared<TriMesh<Scalar>>(non_contact_meshes[id], getVarList());
        feasture_meshes[id]->feature_meshes.push_back(triMesh);
        feasture_meshes[id]->contact_mesh = contact_meshes[id];
    }


    Box<Scalar> box(pts);
    Scalar max_width = max(box.size[0], box.size[1]);
    max_width = max(max_width, box.size[2]);
    Scalar scale_factor = 2.0 / max_width;
    for(int id = 0; id < number_of_parts; id++)
    {

        for(int jd = 0; jd < feasture_meshes[id]->feature_meshes.size(); jd++)
        {
            vector<shared_ptr<Triangle<Scalar>>> &triLists = feasture_meshes[id]->feature_meshes[jd]->triList;
            for(auto &tri: triLists){
                for(int kd = 0; kd < 3; kd++){
                    tri->v[kd] -= box.cenPt;
                    tri->v[kd] *= scale_factor;
                }
            }
        }

        feasture_meshes[id]->contact_mesh->translateMesh(-box.cenPt);

        for(int jd = 0; jd < feasture_meshes[id]->contact_mesh->size(); jd++)
        {
            for(int kd = 0; kd < feasture_meshes[id]->contact_mesh->polyList[jd]->size(); kd++){
                auto &pt = feasture_meshes[id]->contact_mesh->polyList[jd]->vers[kd];
                pt->pos -= box.cenPt;
                pt->pos *= scale_factor;
            }
        }
    }
}


#endif //TOPOLITE_TRIMESHWITHFEATURES_H
