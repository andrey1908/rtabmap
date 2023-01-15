#pragma once

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/LocalMap.h>
#include <rtabmap/core/MapLimits.h>

#include <memory>
#include <optional>
#include <Eigen/Core>

namespace rtabmap {

struct TransformedLocalMap
{
    Transform pose;
    Eigen::Matrix2Xi points;
    MapLimits mapLimits;
};

struct Node
{
    template <typename LocalMapType, typename TransformedLocalMapType>
    Node(
        const std::shared_ptr<LocalMapType>& otherLocalMap,
        TransformedLocalMapType&& otherTransformedLocalMap) :
            localMap(otherLocalMap),
            transformedLocalMap(
                std::forward<TransformedLocalMapType>(otherTransformedLocalMap)) {}
    const std::shared_ptr<const LocalMap> localMap;
    std::optional<TransformedLocalMap> transformedLocalMap;
};

}