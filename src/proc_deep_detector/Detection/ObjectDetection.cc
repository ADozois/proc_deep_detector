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

#include "ObjectDetection.h"

ObjectDetection::ObjectDetection(BoundingBox bbox, std::string class_name, double confidence):
    class_name_{class_name},
    confidence_{confidence} {
    bbox_.TransferBoundingBox(&bbox);
}

ObjectDetection::ObjectDetection(double center_x, double center_y, double width, double height,
                                 std::string class_name, double confidence) :
        class_name_{class_name},
        confidence_{confidence}{
    bbox_.center_.x = center_x;
    bbox_.center_.y = center_y;
    bbox_.center_.theta = 0.0;
    bbox_.width_ = width;
    bbox_.height_ = height;
}

bool ObjectDetection::operator>(const ObjectDetection &object) {
    if (this->confidence_ > object.confidence_){
        return true;
    }
    return false;
}

bool ObjectDetection::operator==(const ObjectDetection &object) {
    if (this->confidence_ == object.confidence_){
        return true;
    }
    return false;
}

const BoundingBox &ObjectDetection::GetBbox_() const {
    return bbox_;
}

const std::string &ObjectDetection::GetClass_name_() const {
    return class_name_;
}

double ObjectDetection::GetConfidence_() const {
    return confidence_;
}
