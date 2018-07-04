//
// Created by walle on 7/3/18.
//

#include "TensorflowModel.h"


TensorflowModel::TensorflowModel(const std::string &graph_path):
    session_(),
    session_status_(),
    graph_(graph_path)
{
    CreateSession();
}

void TensorflowModel::CreateSession() {
    if (graph_.GetStatusGraph() == tensorflow::Status::OK()){
        session_.reset(tensorflow::NewSession(tensorflow::SessionOptions()));
        session_status_ = session_->Create(graph_.GetGraph());

        if (!session_status_.ok()){
            throw std::runtime_error("Can't create session");
        }
    } else{
        throw std::runtime_error("Error with graph: " + graph_.GetStatusGraph().ToString());
    }
}