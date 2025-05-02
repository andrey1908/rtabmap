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

#include <kas_utils/multi_array.hpp>


namespace rtabmap {

using kas_utils::MultiArray;

template <int Dims>
class LocalMap
{
public:
    struct ColoredGrid
    {
        float cellSize = 0.0f;
        MapLimits<int, Dims> limits;
        MultiArray<std::uint8_t, Dims> grid;
        MultiArray<std::int32_t, Dims> colors;

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
    Eigen::Matrix3Xf points_;  // z = 0 for 2d
    std::vector<Color> colors_;
    bool pointsDuplicated_;

    // information about ColoredGrid that produced LocalMap
    float cellSize_;
    MapLimits<int, Dims> limits_;

    Properties properties_;
};

typedef LocalMap<2> LocalMap2d;
typedef LocalMap<3> LocalMap3d;

template <int Dims>
proto::LocalMap::ColoredGrid toProto(const typename LocalMap<Dims>::ColoredGrid& coloredGrid);
template <int Dims>
typename LocalMap<Dims>::ColoredGrid fromProto(const proto::LocalMap::ColoredGrid& proto);

template <int Dims>
proto::LocalMap toProto(const LocalMap<Dims>& localMap);
template <int Dims>
std::shared_ptr<LocalMap<Dims>> fromProto(const proto::LocalMap& proto);

}