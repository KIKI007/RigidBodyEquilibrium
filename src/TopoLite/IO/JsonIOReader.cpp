#include "JsonIOReader.h"
#include <fstream>

//reader

bool JsonIOReader::read()
{
    ifstream filein(input_path, std::ifstream::binary);

    if(filein.fail()) return false;
    std::vector<uint8_t> binary_str((std::istreambuf_iterator<char>(filein)), std::istreambuf_iterator<char>());

    nlohmann::json document;
    document = nlohmann::json::from_ubjson(binary_str);

    if(document.contains("parameter")){
        parseParameters(document["parameter"], data.lock()->varList);
    }

    if(document.contains("reference surface")){
        readReferenceMesh(document["reference surface"]);
    }

    if(document.contains("pattern mesh")){
        readPatternMesh(document["pattern mesh"]);
    }

    if(document.contains("cross mesh")){
        readCrossMesh(document["cross mesh"]);
    }
    return true;
}

bool JsonIOReader::readPatternMesh(const nlohmann::json &mesh_json) {
    data.lock()->pattern_mesh = make_shared<PolyMesh<double>>(data.lock()->varList);
    data.lock()->pattern_mesh->parse(mesh_json);
    return true;
}

bool JsonIOReader::readReferenceMesh(const nlohmann::json &mesh_json) {
    data.lock()->reference_surface = make_shared<PolyMesh<double>>(data.lock()->varList);
    data.lock()->reference_surface->parse(mesh_json);
    return true;
}

bool JsonIOReader::readCrossMesh(const nlohmann::json &mesh_json) {
    data.lock()->cross_mesh = make_shared<CrossMesh<double>>(data.lock()->varList);
    data.lock()->cross_mesh->parse(mesh_json);
    return true;
}
