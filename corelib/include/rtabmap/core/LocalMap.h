#pragma once

#include <rtabmap/core/Time.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Color.h>
#include <rtabmap/core/MapLimits.h>
#include <rtabmap/core/Grid.h>

#include <vector>
#include <memory>
#include <optional>
#include <utility>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include <rtabmap/proto/Time.pb.h>
#include <rtabmap/proto/Color.pb.h>
#include <rtabmap/proto/Transform.pb.h>
#include <rtabmap/proto/LocalMap.pb.h>

namespace rtabmap {

class LocalMap
{
public:
    struct ColoredGrid
    {
        float cellSize = 0.0f;
        MapLimitsI limits;
        cv::Mat grid;  // CV_8S
        cv::Mat colors;  // CV_32S

        static constexpr std::int8_t unknownCellValue = -1;
        static constexpr std::int8_t emptyCellValue = 0;
        static constexpr std::int8_t occupiedCellValue = 100;
    };

public:
    LocalMap() = default;
    LocalMap(const ColoredGrid& coloredGrid,
        float maxRange2dSqr, bool duplicatePoints);
    ~LocalMap() = default;

    void fromColoredGrid(const ColoredGrid& coloredGrid,
        float maxRange2dSqr, bool duplicatePoints);
    ColoredGrid toColoredGrid() const;

    bool isObstacle(int i) const { return i < numObstacles_; }

    int numObstacles() const { return numObstacles_; }
    int numEmpty() const { return numEmpty_; }
    const Eigen::Matrix3Xf& points() const { return points_; }
    const std::vector<Color>& colors() const { return colors_; }

    void setSensorBlindRange2dSqr(float sensorBlindRange2dSqr)
        { sensorBlindRange2dSqr_ = sensorBlindRange2dSqr; }
    float sensorBlindRange2dSqr() const { return sensorBlindRange2dSqr_; }
    template<typename T>
    void setToSensor(T&& toSensor) { toSensor_ = std::forward<T>(toSensor); }
    const Transform& toSensor() const { return toSensor_; }

    template<typename T>
    void setFromUpdatedPose(T&& fromUpdatedPose)
        { fromUpdatedPose_ = std::forward<T>(fromUpdatedPose); }
    const Transform& fromUpdatedPose() const { return fromUpdatedPose_; }

    void setTime(const Time& time) { time_ = time; }
    const Time& time() const { return time_; }

    bool pointsDuplicated() const { return pointsDuplicated_; }

private:
    int numObstacles_;
    int numEmpty_;
    Eigen::Matrix3Xf points_;  // z = 0
    std::vector<Color> colors_;

    float sensorBlindRange2dSqr_;
    Transform toSensor_;

    // used to correct poses passed to updatePoses() function
    Transform fromUpdatedPose_;

    Time time_;

    bool pointsDuplicated_;

    // information about ColoredGrid that produced LocalMap
    float cellSize_;
    MapLimitsI limits_;
};

proto::LocalMap::ColoredGrid toProto(const LocalMap::ColoredGrid& coloredGrid);
LocalMap::ColoredGrid fromProto(const proto::LocalMap::ColoredGrid& proto);

proto::LocalMap toProto(const LocalMap& localMap);
std::shared_ptr<LocalMap> fromProto(const proto::LocalMap& proto);

}