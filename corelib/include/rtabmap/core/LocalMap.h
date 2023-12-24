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
        cv::Mat grid;  // CV_8U
        cv::Mat colors;  // CV_32S

        static constexpr std::uint8_t unknownCellValue = 0;
        static constexpr std::uint8_t occupiedCellValue = 2;
        static constexpr std::uint8_t maybeEmptyCellValue = 4;
        static constexpr std::uint8_t emptyCellValue = 1;

        static constexpr std::uint8_t ignoredOccupiedCellValue = 3;
    };

    struct Properties
    {
        // used to correct poses passed to updatePoses() function
        Transform fromUpdatedPose = Transform::getIdentity();

        Time time = Time(0, 0);
    };

    enum PointType
    {
        Occupied,
        MaybeEmpty,
        Empty
    };

public:
    LocalMap();
    LocalMap(const Properties& properties);
    LocalMap(const ColoredGrid& coloredGrid,
        float maxRange2dSqr, bool duplicatePoints);
    LocalMap(const ColoredGrid& coloredGrid,
        float maxRange2dSqr, bool duplicatePoints,
        const Properties& properties);

    ~LocalMap() = default;

    void fromColoredGrid(const ColoredGrid& coloredGrid,
        float maxRange2dSqr, bool duplicatePoints);
    ColoredGrid toColoredGrid() const;

    template<typename T>
    void setProperties(T&& properties) { properties_ = std::forward<T>(properties); }
    const Properties& properties() const { return properties_; }

    PointType getPointType(int i) const
    {
        if (i < numObstacles_)
        {
            return PointType::Occupied;
        }
        if (i < numMaybeEmpty_)
        {
            return PointType::MaybeEmpty;
        }
        return PointType::Empty;
    }

    int numObstacles() const { return numObstacles_; }
    int numMaybeEmpty() const { return numMaybeEmpty_; }
    int numEmpty() const { return numEmpty_; }
    const Eigen::Matrix3Xf& points() const { return points_; }
    const std::vector<Color>& colors() const { return colors_; }
    bool pointsDuplicated() const { return pointsDuplicated_; }

    template<typename T>
    void setFromUpdatedPose(T&& fromUpdatedPose)
        { properties_.fromUpdatedPose = std::forward<T>(fromUpdatedPose); }
    const Transform& fromUpdatedPose() const { return properties_.fromUpdatedPose; }

    void setTime(const Time& time) { properties_.time = time; }
    const Time& time() const { return properties_.time; }

private:
    int numObstacles_;
    int numMaybeEmpty_;
    int numEmpty_;
    Eigen::Matrix3Xf points_;  // z = 0
    std::vector<Color> colors_;
    bool pointsDuplicated_;

    // information about ColoredGrid that produced LocalMap
    float cellSize_;
    MapLimitsI limits_;

    Properties properties_;
};

proto::LocalMap::ColoredGrid toProto(const LocalMap::ColoredGrid& coloredGrid);
LocalMap::ColoredGrid fromProto(const proto::LocalMap::ColoredGrid& proto);

proto::LocalMap toProto(const LocalMap& localMap);
std::shared_ptr<LocalMap> fromProto(const proto::LocalMap& proto);

}