#pragma once

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/LocalMapBuilder.h>
#include <rtabmap/core/Grid.h>
#include <rtabmap/core/MapLimits.h>
#include <rtabmap/core/Color.h>
#include <rtabmap/core/Node.h>

#include <deque>
#include <map>
#include <utility>
#include <optional>
#include <Eigen/Core>

namespace rtabmap {

class TemporaryOccupancyGridBuilder
{
private:
    using CounterType = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    using ColorsType = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

public:
    TemporaryOccupancyGridBuilder(const ParametersMap& parameters = ParametersMap());
    void parseParameters(const ParametersMap& parameters);

    void addLocalMap(const Transform& pose, std::shared_ptr<const LocalMap> localMap);

    void updatePoses(const std::deque<Transform>& updatedPoses);

    OccupancyGrid getOccupancyGrid() const;
    OccupancyGrid getOccupancyGrid(const MapLimitsI& roi) const;
    OccupancyGrid getProbOccupancyGrid() const;
    OccupancyGrid getProbOccupancyGrid(const MapLimitsI& roi) const;
    ColorGrid getColorGrid() const;
    ColorGrid getColorGrid(const MapLimitsI& roi) const;

    int maxTemporaryLocalMaps() const { return maxTemporaryLocalMaps_; }
    const std::deque<Node>& nodes() const { return nodes_; }
    const MapLimitsI& mapLimits() const { return mapLimits_; }

    void reset();

private:
    TransformedLocalMap transformLocalMap(const LocalMap& localMap, const Transform& transform);
    void createOrResizeMap(const MapLimitsI& newMapLimits);
    void deployLastLocalMap();
    void deployLocalMap(const Node& node);
    void removeLocalMap();

    void clear();

private:
    float cellSize_;
    float temporaryMissProb_;
    float temporaryHitProb_;
    float temporaryOccupancyProbThr_;
    int updated_;
    int maxTemporaryLocalMaps_;

    std::deque<Node> nodes_;
    MapLimitsI mapLimits_;
    CounterType hitCounter_;
    CounterType missCounter_;
    ColorsType colors_;

    Eigen::Matrix<std::int8_t, Eigen::Dynamic, Eigen::Dynamic> probabilities_;
    Eigen::Matrix<std::int8_t, Eigen::Dynamic, Eigen::Dynamic> probabilitiesThr_;
};

}
