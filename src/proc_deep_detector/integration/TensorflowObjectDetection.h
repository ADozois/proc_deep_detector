//
// Created by walle on 7/4/18.
//

#ifndef TENSOR_TENSORFLOWOBJECTDETECTION_H
#define TENSOR_TENSORFLOWOBJECTDETECTION_H

#include "TensorflowModel.h"
#include <vector>
#include "opencv2/core/core.hpp"

class TensorflowObjectDetection: public TensorflowModel {
public:
    TensorflowObjectDetection() = delete;
    TensorflowObjectDetection(const std::string &graph_path, std::string input_node, std::string output_node);
    TensorflowObjectDetection(const std::string &config_path);
    ~TensorflowObjectDetection() = default;

    std::vector<tensorflow::Tensor> Run(tensorflow::Tensor &img);
    std::vector<tensorflow::Tensor> Run(std::vector<tensorflow::Tensor> &img);
    std::vector<tensorflow::Tensor> Run(cv::Mat &img);


private:
    std::vector<tensorflow::Tensor> input_;
    std::vector<tensorflow::Tensor> output_;
    std::string input_node_;
    std::string output_node_;

    void ParseConfigFile(const std::string &config_path);

};


#endif //TENSOR_TENSORFLOWOBJECTDETECTION_H
