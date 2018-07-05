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

#include "TensorflowObjectDetection.h"


TensorflowObjectDetection::TensorflowObjectDetection(const std::string &config_path) {

}

TensorflowObjectDetection::TensorflowObjectDetection(const std::string &graph_path, std::string input_node,
                                                     std::string output_node):
    TensorflowModel(graph_path),
    input_node_{input_node},
    output_node_{output_node} {

}

void TensorflowObjectDetection::ParseConfigFile(const std::string &config_path) {

}