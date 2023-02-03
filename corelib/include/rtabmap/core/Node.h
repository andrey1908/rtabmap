#pragma once

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/LocalMap.h>
#include <rtabmap/core/MapLimits.h>

#include <memory>
#include <Eigen/Core>

namespace rtabmap {

class TransformedLocalMap
{
public:
    TransformedLocalMap() = default;
    TransformedLocalMap(const LocalMap& localMap, const Transform& pose, float cellSize)
    {
        set(localMap, pose, cellSize);
    }

    void set(const LocalMap& localMap, const Transform& pose, float cellSize);

    bool valid() const { return mapLimits_.valid(); }
    void clear()
    {
        points_ = Eigen::Matrix2Xi();
        mapLimits_ = MapLimitsI();
    }

    const Transform& pose() const { return pose_; }
    const Eigen::Matrix2Xi& points() const { return points_; }
    const MapLimitsI& mapLimits() const { return mapLimits_; }

private:
    Transform pose_;
    Eigen::Matrix2Xi points_;
    MapLimitsI mapLimits_;
};

struct Node
{
    template <typename T, typename U>
    Node(const std::shared_ptr<T>& otherLocalMap, U&& otherTransformedLocalMap) :
        localMap(otherLocalMap),
        transformedLocalMap(std::forward<U>(otherTransformedLocalMap)) {}
    const std::shared_ptr<const LocalMap> localMap;
    TransformedLocalMap transformedLocalMap;
};

}