/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/utilite/ULogger.h>

#include <rtabmap/proto/SensorData.pb.h>

#include <opencv2/core/core.hpp>
#include <Eigen/Dense>

#include <vector>
#include <optional>
#include <string>

namespace rtabmap
{

class SensorData
{
public:
    struct Pixel
    {
        int u;
        int v;
    };

    struct CameraParameters
    {
        Pixel project(float x, float y, float z) const
        {
            Pixel pixel{-1, -1};
            if (z <= 0.0f)
            {
                return pixel;
            }
            pixel.u = fx * x / z + cx;
            pixel.v = fy * y / z + cy;
            return pixel;
        }
        bool inFrame(const Pixel& pixel) const
        {
            const int& u = pixel.u;
            const int& v = pixel.v;
            return u >= 0 && v >= 0 && u < width && v < height;
        }

        int width;
        int height;
        float fx;
        float fy;
        float cx;
        float cy;
    };

    struct CameraData
    {
        Transform toSensor;
        CameraParameters parameters;
        cv::Mat image;
    };

    struct LidarData
    {
        Transform toSensor;
        Eigen::Matrix3Xf points;
    };

public:
    SensorData() = default;

    template<typename T>
    void addCameraData(T&& cameraData)
    {
        camerasData_.push_back(std::forward<T>(cameraData));
    }
    int numCamerasData() const { return camerasData_.size(); }
    void clearCamerasData() { camerasData_.clear(); }
    const std::vector<CameraData>& camerasData() const { return camerasData_; }

    template<typename T>
    void addLidarData(T&& lidarData)
    {
        lidarsData_.push_back(std::forward<T>(lidarData));
    }
    int numLidarsData() const { return lidarsData_.size(); }
    void clearLidarsData() { lidarsData_.clear(); }
    const std::vector<LidarData>& lidarsData() const { return lidarsData_; }

private:
    std::vector<CameraData> camerasData_;
    std::vector<LidarData> lidarsData_;
};

proto::CameraParameters toProto(const SensorData::CameraParameters& parameters);
SensorData::CameraParameters fromProto(const proto::CameraParameters& proto);

proto::CameraImage toProto(const cv::Mat& image);
cv::Mat fromProto(const proto::CameraImage& proto);

proto::CameraData toProto(const SensorData::CameraData& cameraData);
SensorData::CameraData fromProto(const proto::CameraData& proto);

proto::LidarData toProto(const SensorData::LidarData& lidarData);
SensorData::LidarData fromProto(const proto::LidarData& proto);

proto::SensorData toProto(const SensorData& sensorData);
SensorData fromProto(const proto::SensorData& proto);

}
