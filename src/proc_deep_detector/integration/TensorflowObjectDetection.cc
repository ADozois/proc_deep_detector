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
    tensorflow::Tensor image_input = tensorflow::Tensor(tensorflow::DT_FLOAT, tensorflow::TensorShape(
            {1, img.size().height, img.size().width, img.channels()}));
    auto input_tensor_mapped = image_input.tensor<float, 4>();
    cv::Mat image_float;
    img.convertTo(image_float, CV_32FC1);
    const float *source_data = (float *) image_float.data;

    for (int y = 0; y < img.size().height; ++y) {
        const float *source_row = source_data + (y * img.size().height * img.channels());
        for (int x = 0; x < img.size().width; ++x) {
            const float *source_pixel = source_row + (x * img.channels());
            for (int c = 0; c < img.channels(); ++c) {
                const float *source_value = source_pixel + c;
                input_tensor_mapped(0, y, x, c) = *source_value;
            }
        }
    }
    return image_input;
}

void TensorflowObjectDetection::AddImage(cv::Mat &img) {
    input_.push_back(ImageToTensor(img));
}
