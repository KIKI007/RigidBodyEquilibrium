//
// Created by ziqwang on 05.06.20.
//

#include "JsonIOWriter.h"
#include <fstream>

void JsonIOWriter::write()
{
    nlohmann::json result;
    if(data.lock()){
        //1) parameter
        {
            result["parameter"] = getParameterJson(data.lock()->varList);
        }

        //2) reference surface
        if(data.lock()->reference_surface){
            result["reference surface"] = data.lock()->reference_surface->dump();
        }

        //3) pattern mesh
        if(data.lock()->pattern_mesh){
            result["pattern mesh"] = data.lock()->pattern_mesh->dump();
        }

        //4) cross mesh
        if(data.lock()->cross_mesh){
            result["cross mesh"] = data.lock()->cross_mesh->dump();
        }
    }

    std::ofstream fileout(output_path, std::ofstream::binary);
    std::vector<std::uint8_t> binary_result = nlohmann::json::to_ubjson(result);
    if(fileout){
        fileout.write((char *)binary_result.data(), binary_result.size());
    }
}
