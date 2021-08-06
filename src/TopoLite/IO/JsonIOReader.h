//
// Created by ziqwang on 29.10.18.
//

#ifndef TOPOLOCKCREATOR_GLUIXML_H
#define TOPOLOCKCREATOR_GLUIXML_H

#include "TopoLite/Utility/HelpDefine.h"
#include "IO/IOData.h"

#include <string>
#include <nlohmann/json.hpp>

#if defined(GCC_VERSION_LESS_8)
#include <experimental/filesystem>
    using namespace std::experimental::filesystem;
#else
#include <filesystem>
using namespace std::filesystem;
#endif

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

class JsonIOReader
{
public:

    path input_path;
    weak_ptr<IOData> data;


public:

    JsonIOReader(const std::string _input_path, shared_ptr<IOData> _data){
        input_path = _input_path;
        data = _data;
    }

public:

    bool read();

    bool readPatternMesh(const nlohmann::json& mesh_json);

    bool readReferenceMesh(const nlohmann::json& mesh_json);

    bool readCrossMesh(const nlohmann::json& mesh_json);
};

static bool parseParameters(const nlohmann::json &parameter_json, shared_ptr<InputVarList> &varList, bool initPara = true)
{
    if(initPara){
        varList = make_shared<InputVarList>();
        InitVar(varList.get());
    }

    InputVarManager manager;
    for(auto& [key, value] : parameter_json.items()){
        shared_ptr<InputVar> var = manager.readJSON(value, key);
        varList->add(var);
    }
    return true;
}

template <int dim, typename Scalar>
static Matrix<Scalar, dim, 1> parseEigenVector(nlohmann::json json){
    vector<Scalar> data = json.get<vector<Scalar>>();
    if(data.size() != dim) {
        return Matrix<Scalar, dim, 1>();
    }

    Matrix<Scalar, dim, 1> vec;
    for(int id = 0; id < dim; id++){
        vec(id) = data[id];
    }

    return vec;
}

template <int dim, typename Scalar>
static vector<Matrix<Scalar, dim, 1>> parseEigenVectorList(nlohmann::json json){
    vector<Scalar> data = json.get<vector<Scalar>>();
    vector<Matrix<Scalar, dim, 1>> vecs;
    if(data.size() % dim != 0) {
        return vector<Matrix<Scalar, dim, 1>>();
    }

    for(int id = 0; id < data.size() / dim; id++){
        Matrix<Scalar, dim, 1> vec;
        for(int jd = 0; jd < dim; jd++){
            vec(jd) = data[id * dim + jd];
        }
        vecs.push_back(vec);
    }
    return vecs;
}


#endif //TOPOLOCKCREATOR_GLUIXML_H