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

#ifndef TENSOR_TENSORFLOWOBJECTDETECTION_H
#define TENSOR_TENSORFLOWOBJECTDETECTION_H

#include "TensorflowModel.h"
#include <vector>
#include "opencv2/core/core.hpp"
#include "../Detection/ObjectDetection.h"

class TensorflowObjectDetection: public TensorflowModel::TensorflowModel {
public:
    TensorflowObjectDetection() = delete;
    TensorflowObjectDetection(const std::string &graph_path, std::string input_node,
                              std::vector<std::string> output_node);
    ~TensorflowObjectDetection() = default;

    void Run(tensorflow::Tensor &img);
    void Run() override;
    std::vector<tensorflow::Tensor> Run(cv::Mat &img);

    void AddImage(cv::Mat &img);

    TensorflowObjectDetection & operator=(const TensorflowObjectDetection &object);

private:
    std::vector<tensorflow::Tensor> input_;
    std::vector<tensorflow::Tensor> output_;
    std::string input_node_;
    std::vector<std::string> output_node_;
    std::vector<ObjectDetection> prediction_;
    double threshold_;

    void FilterOutput();

    tensorflow::Tensor ImageToTensor(cv::Mat &img);
};


#endif //TENSOR_TENSORFLOWOBJECTDETECTION_H
