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

#ifndef TENSOR_TENSORFLOWSESSION_H
#define TENSOR_TENSORFLOWSESSION_H

#include <tensorflow/core/public/session.h>
#include "TensorflowGraph.h"


class TensorflowModel {
public:
    TensorflowModel() = delete;
    explicit TensorflowModel(const std::string &graph_path);
    virtual ~TensorflowModel() = default;

    virtual void Run() = 0;


protected:
    std::unique_ptr<tensorflow::Session>  session_;
    TensorflowGraph graph_;
    tensorflow::Status session_status_;

    void CreateSession();
};


#endif //TENSOR_TENSORFLOWSESSION_H
