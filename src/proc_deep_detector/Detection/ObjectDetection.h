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

#ifndef PROC_DEEP_DETECTOR_OBJECTDETECTION_H
#define PROC_DEEP_DETECTOR_OBJECTDETECTION_H

#include "BoundingBox.h"


class ObjectDetection {
public:
    ObjectDetection() = default;
    ~ObjectDetection() = default;
    ObjectDetection(BoundingBox bbox, std::string class_name, double confidence);
    ObjectDetection(double center_x, double center_y, double width, double height,
                        std::string class_name, double confidence);

    bool operator>(const ObjectDetection &object);
    bool operator==(const ObjectDetection &object);

    const BoundingBox &GetBbox_() const;

    const std::string &GetClass_name_() const;

    double GetConfidence_() const;

private:
    BoundingBox bbox_;
    std::string class_name_;
    double confidence_;

};


#endif //PROC_DEEP_DETECTOR_OBJECTDETECTION_H
