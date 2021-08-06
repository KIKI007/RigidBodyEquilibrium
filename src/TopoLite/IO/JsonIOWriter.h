//
// Created by ziqwang on 05.06.20.
//

#ifndef TOPOLITE_JSONIOWRITER_H
#define TOPOLITE_JSONIOWRITER_H

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

class JsonIOWriter{
public:
    path output_path;
    weak_ptr<IOData> data;

public:

    JsonIOWriter(const std::string _output_path, shared_ptr<IOData> _data){
        output_path = _output_path;
        data = _data;
    }

    void write();
};

template<int dim, typename Scalar>
static nlohmann::json dumpEigenVector(Matrix<Scalar, dim, 1> vec){
    nlohmann::json json;
    vector<double> values;
    for(int id = 0; id < dim; id++){
        values.push_back(vec(id));
    }
    json = values;
    return json;
}

template<int dim, typename Scalar>
static nlohmann::json dumpEigenVectorList(const vector<Matrix<Scalar, dim, 1>> &vecs) {
    vector<Scalar> values;

    for(int id = 0; id < vecs.size(); id++)
    {
        for(int jd = 0; jd < dim; jd++){
            values.push_back(vecs[id](jd));
        }
    }

    nlohmann::json json = values;
    return json;
}


static nlohmann::json getParameterJson(shared_ptr<InputVarList> data){
    nlohmann::json parameter_json;
    InputVarManager manager;

    for(shared_ptr<InputVar> var: data->varLists){
        std::string name;
        nlohmann::json var_json;
        std::tie(var_json, name) = manager.getJSON(var.get());
        parameter_json[name] = var_json;
    }

    return parameter_json;
}

#endif //TOPOLITE_JSONIOWRITER_H
