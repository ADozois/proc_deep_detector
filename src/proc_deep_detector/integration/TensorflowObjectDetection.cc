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
#include <fstream>
#include <regex>
#include <iostream>
#include <utility>
#include <boost/algorithm/string.hpp>


namespace proc_deep_detector {


TensorflowObjectDetection::TensorflowObjectDetection(const std::string &graph_path, const std::string &label_path, std::string input_node,
                                                     std::vector<std::string> output_node, float threshold)
        :
        TensorflowModel(graph_path),
        input_node_{input_node},
        output_node_{output_node},
        img_width_{0},
        img_height_{0},
        img_depth_{0},
        threshold_{threshold}{
    ParseLabel(label_path);
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
    FilterOutput();
    input_.erase(input_.begin());
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
    float *tensor_data_ptr = image_input.flat<float>().data();
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
    img_width_ = img.size().width;
    img_height_ = img.size().height;
    input_.push_back(ImageToTensor(img));
}

void TensorflowObjectDetection::FilterOutput() {
    prediction_.clear();
    tensorflow::TTypes<float, 3>::Tensor boxes = output_[0].flat_inner_dims<float, 3>();
    tensorflow::TTypes<float>::Flat scores = output_[1].flat<float>();
    tensorflow::TTypes<float>::Flat classes = output_[2].flat<float>();
    tensorflow::TTypes<float>::Flat numDetections = output_[3].flat<float>();

    proc_deep_detector::Detection object;

    for (int i = 0; i < scores.size(); ++i) {
        float test = scores(i);
        if (scores(i) >= threshold_) {
            object.bbox = CreateBoundingBox(boxes, i);
            object.confidence = scores(i);
            object.class_name.data = ConvertIdToName((int)(classes(i)));
            if (!object.class_name.data.empty()){
                prediction_.push_back(object);
            }
        }
    }
}

BoundingBox2D
TensorflowObjectDetection::CreateBoundingBox(tensorflow::TTypes<float, 3>::Tensor &boxes, int index) {
    proc_deep_detector::BoundingBox2D bbox;
    float top = boxes(0,index,0) * img_height_;
    float left = boxes(0,index,1) * img_width_;
    float bottom = boxes(0,index,2) * img_height_;
    float right = boxes(0,index,3) * img_width_;
    bbox.center = CalculateCenter(left, right, top, bottom);
    bbox.size_x = right - left;
    bbox.size_y = bottom - top;
    return bbox;
}

geometry_msgs::Pose2D
TensorflowObjectDetection::CalculateCenter(float &left, float &right, float &top, float &bottom) {
    geometry_msgs::Pose2D center;
    center.theta = 0.0;
    center.x = left + (right - left)/2.0;
    center.y = top + (bottom - top)/2.0;
    return center;
}

geometry_msgs::Pose2D TensorflowObjectDetection::CalculateSize(tensorflow::TTypes<float, 3>::Tensor &boxe) {
    return geometry_msgs::Pose2D();
}

void TensorflowObjectDetection::ParseLabel(const std::string &label_path) {
    std::ifstream file(label_path);
    if(file.bad()){
        throw std::runtime_error("Can't read label file");
    }

    std::stringstream ss;
    ss << file.rdbuf();
    std::string file_data = ss.str();

    std::smatch matcher_entry;
    const std::regex reg_entry("item \\{([\\S\\s]*?)\\}");
    int id;
    std::string name;

    for (std::sregex_iterator it = std::sregex_iterator(file_data.begin(), file_data.end(), reg_entry); it != std::sregex_iterator(); ++it) {
        matcher_entry = *it;
        id = ExtractId(matcher_entry.str());
        name = ExtractName(matcher_entry.str());
        label_map_.insert(std::pair<int, std::string>(id, name));
    }




}

int TensorflowObjectDetection::ExtractId(const std::string &line) {
    const std::regex reg_id("[0-9]+");
    std::smatch matcher_id;
    std::regex_search(line, matcher_id, reg_id);
    std::string result = matcher_id.str();
    return std::stoi(result);
}

std::string TensorflowObjectDetection::ExtractName(const std::string &line) {
    const std::regex reg_name("\".+\"");
    std::smatch matcher_name;
    std::regex_search(line, matcher_name, reg_name);
    std::string result =  matcher_name.str();
    boost::erase_all(result, "\"");
    return result;
}

std::string TensorflowObjectDetection::ConvertIdToName(int id) {
    std::string name;
    if (id <= label_map_.size() + 1){
        name = label_map_.find(id)->second;
    }
    return name;
}

    std::vector<Detection> TensorflowObjectDetection::GetPredictions() {
        return prediction_;
    }

}