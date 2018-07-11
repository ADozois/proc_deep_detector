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



#include "tensorflow/cc/ops/const_op.h"
#include "tensorflow/cc/ops/image_ops.h"
#include "tensorflow/cc/ops/standard_ops.h"
#include "tensorflow/core/framework/graph.pb.h"
#include "tensorflow/core/framework/tensor.h"
#include "tensorflow/core/graph/default_device.h"
#include "tensorflow/core/graph/graph_def_builder.h"
#include "tensorflow/core/lib/core/errors.h"
#include "tensorflow/core/lib/core/stringpiece.h"
#include "tensorflow/core/lib/core/threadpool.h"
#include "tensorflow/core/lib/io/path.h"
#include "tensorflow/core/lib/strings/stringprintf.h"
#include "tensorflow/core/platform/env.h"
#include "tensorflow/core/platform/init_main.h"
#include "tensorflow/core/platform/logging.h"
#include "tensorflow/core/platform/types.h"
#include "tensorflow/core/public/session.h"
#include "tensorflow/core/util/command_line_flags.h"

#include "TensorflowObjectDetection.h"
#include <utility>


TensorflowObjectDetection::TensorflowObjectDetection(const std::string &graph_path, std::string input_node,
                                                     std::vector<std::string> output_node) :
        TensorflowModel(graph_path),
        input_node_{input_node},
        output_node_{output_node} {

}

void TensorflowObjectDetection::Run(tensorflow::Tensor &img) {
    std::pair<std::string, tensorflow::Tensor> input_pair = std::make_pair(input_node_, img);
    std::vector<std::pair<std::string, tensorflow::Tensor>> inference_input_;
    inference_input_.push_back(input_pair);
    session_->Run(inference_input_, output_node_, {}, &output_);
}

void TensorflowObjectDetection::Run() {
    std::pair<std::string, tensorflow::Tensor> input_pair = std::make_pair(input_node_, input_.front());
    std::vector<std::pair<std::string, tensorflow::Tensor>> inference_input_;
    inference_input_.push_back(input_pair);
    session_->Run(inference_input_, output_node_, {}, &output_);
    if (!output_.empty()){
        std::cout << "Object detected" << std::endl;
    } else{
        std::cout << "Nothing detected" << std::endl;
    }
}

std::vector<tensorflow::Tensor> TensorflowObjectDetection::Run(cv::Mat &img) {

}

TensorflowObjectDetection &TensorflowObjectDetection::operator=(const TensorflowObjectDetection &object) {
    this->input_ = object.input_;
    this->output_ = object.output_;
    this->input_node_ = object.input_node_;
    this->output_node_ = object.output_node_;
    this->threshold_ = object.threshold_;
    this->prediction_ = object.prediction_;
    return *this;
}

tensorflow::Tensor TensorflowObjectDetection::ImageToTensor(cv::Mat &img) {

    auto root = tensorflow::Scope::NewRootScope();
    using namespace ::tensorflow::ops;

    tensorflow::Tensor image_input = tensorflow::Tensor(tensorflow::DT_FLOAT, tensorflow::TensorShape(
            {1, img.size().height, img.size().width, img.channels()}));
    float* tensor_data_ptr = image_input.flat<float>().data();
    cv::Mat fake_mat(img.rows, img.cols, CV_32FC3, tensor_data_ptr);
    img.convertTo(fake_mat, CV_32FC3);

    auto input_tensor = Placeholder(root.WithOpName("input"), tensorflow::DT_FLOAT);
    std::vector<std::pair<std::string, tensorflow::Tensor>> inputs = {{"input", image_input}};
    auto uint8Caster = Cast(root.WithOpName("uint8_Cast"), image_input, tensorflow::DT_UINT8);

    tensorflow::GraphDef graph;
    root.ToGraphDef(&graph);

    std::vector<tensorflow::Tensor> outTensors;
    std::unique_ptr<tensorflow::Session> session(tensorflow::NewSession(tensorflow::SessionOptions()));

    session->Create(graph);
    session->Run({inputs}, {"uint8_Cast"}, {}, &outTensors);

    image_input = outTensors.at(0);

    return image_input;

}

void TensorflowObjectDetection::AddImage(cv::Mat &img) {
    input_.push_back(ImageToTensor(img));
}
