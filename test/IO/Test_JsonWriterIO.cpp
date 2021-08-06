//
// Created by ziqwang on 2020-03-01.
//

#include <catch2/catch.hpp>
#include "IO/JsonIOWriter.h"

#if defined(GCC_VERSION_LESS_8)
#include <experimental/filesystem>
    using namespace std::experimental::filesystem;
#else
#include <filesystem>
using namespace std::filesystem;
#endif

using json = nlohmann::json;

TEST_CASE("Check InputVarManager Json Write"){
    shared_ptr<IOData> data = make_shared<IOData>();
    InitVar(data->varList.get());
    InputVarManager manager;
    std::string name;
    json node;
    std::tie(node, name) = manager.getJSON(data->varList->find("tiltAngle"));
    json dict;
    dict[name] = node;
}

TEST_CASE("Test Write Parameter"){
    shared_ptr<IOData> data = make_shared<IOData>();
    InitVar(data->varList.get());
    path jsonFileName(UNITTEST_DATAPATH);
    jsonFileName = jsonFileName / "TopoInterlock/Json/origin.json";
    std::string path = jsonFileName;
    JsonIOWriter writer(path, data);
    writer.write();
}

TEST_CASE("Test Write Mesh"){
    shared_ptr<IOData> data = make_shared<IOData>();
    InitVar(data->varList.get());
    shared_ptr<PolyMesh<double>> mesh;
    mesh = make_shared<PolyMesh<double>>(data->varList);
    path dataFolder(UNITTEST_DATAPATH);
    path filepath = dataFolder / "Mesh/primitives/Icosphere.obj";
    mesh->readOBJModel(filepath.string().c_str(), false);
    mesh->dump().dump(4);
}