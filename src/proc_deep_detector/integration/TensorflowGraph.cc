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

#include <tensorflow/core/platform/env.h>
#include "TensorflowGraph.h"
#include <unistd.h>

void TensorflowGraph::LoadGraph(std::string graph_path) {
    if(!FileExist(graph_path)){
        throw std::invalid_argument("File not found");
    }
    status_graph = tensorflow::ReadBinaryProto(tensorflow::Env::Default(), graph_path, &graph);
    if(!status_graph.ok()){
        throw std::runtime_error("Can't load graph_ provided");
    }
}

bool TensorflowGraph::FileExist(const std::string &file_path) {
    return ( access( file_path.c_str(), F_OK ) != -1 );
}

TensorflowGraph::TensorflowGraph(const std::string &graph_path) {
    LoadGraph(graph_path);
}

const tensorflow::GraphDef &TensorflowGraph::GetGraph() const {
    return graph;
}

const tensorflow::Status &TensorflowGraph::GetStatusGraph() const {
    return status_graph;
}
