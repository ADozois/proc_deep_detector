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

#ifndef TENSOR_TENSORFLOWGRAPH_H
#define TENSOR_TENSORFLOWGRAPH_H

#include <tensorflow/core/framework/graph.pb.h>
#include <tensorflow/core/lib/core/status.h>


class TensorflowGraph {
public:
    TensorflowGraph() = delete;
    explicit TensorflowGraph(const std::string &graph_path);
    ~TensorflowGraph() = default;
    const tensorflow::GraphDef &GetGraph() const;
    const tensorflow::Status &GetStatusGraph() const;

private:
    tensorflow::Status status_graph;
    tensorflow::GraphDef graph;
    void LoadGraph(std::string graph_path);
    bool FileExist(const std::string &file_path);
};


#endif //TENSOR_TENSORFLOWGRAPH_H
