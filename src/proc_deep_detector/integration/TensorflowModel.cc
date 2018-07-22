/// \author	Antoine Dozois <dozois.a@gmail.com>
/// \copyright Copyright (c) 2018 S.O.N.I.A. All rights reserved.
/// \section LICENSE
/// This file is part of S.O.N.I.A. software.
///
/// S.O.N.I.A. software is free software: you can redistribute it and/or modify
/// it under the terms of the GNU General Public License as published by
/// the Free Software Foundation, either version 3 of the License, or
/// (at your option) any later version.
///
/// S.O.N.I.A. software is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
/// GNU General Public License for more details.
///
/// You should have received a copy of the GNU General Public License
/// along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.

#include "TensorflowModel.h"
#include <algorithm>

namespace proc_deep_detector {

TensorflowModel::TensorflowModel(const std::string &graph_path) :
        session_(),
        session_status_(),
        graph_(graph_path) {
    CreateSession();
}

void TensorflowModel::CreateSession() {
    if (graph_.GetStatusGraph() == tensorflow::Status::OK()) {
        session_.reset(tensorflow::NewSession(tensorflow::SessionOptions()));
        session_status_ = session_->Create(graph_.GetGraph());

        if (!session_status_.ok()) {
            throw std::runtime_error("Can't create session");
        }
    } else {
        throw std::runtime_error("Error with graph: " + graph_.GetStatusGraph().ToString());
    }
}

    ModelType TensorflowModel::GetModelType(const std::string &name) {
        //std::transform(name.begin(), name.end(), name.begin(), ::tolower);
        ModelType type;
        if(name == "detection"){
            type  = ModelType::DETECTION;
        } else if(name == "classification"){
            type = ModelType::CLASSIFICATION;
        } else if(name == "regression"){
            type = ModelType::REGRESSION;
        } else{
            throw std::runtime_error("Type of model: " + name + " not supported");
        }
        return type;
    }

    ModelType TensorflowModel::GetModelType(const int &type_number) {
        ModelType type;
        switch(type_number){
            case 0:
                type  = ModelType::REGRESSION;
                break;
            case 1:
                type  = ModelType::CLASSIFICATION;
                break;
            case 2:
                type  = ModelType::DETECTION;
                break;
            default:
                throw std::runtime_error("Type of model not supported");
        }
        return type;
    }

}