#pragma once

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/LocalMapBuilder.h>
#include <rtabmap/core/Grid.h>
#include <rtabmap/core/MapLimits.h>
#include <rtabmap/core/Color.h>
#include <rtabmap/core/Node.h>

#include <list>
#include <map>
#include <utility>
#include <optional>
#include <Eigen/Core>

namespace rtabmap {

class OccupancyGridBuilder
{
private:
    using MapType = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    using ColorsType = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

public:
    OccupancyGridBuilder(const ParametersMap& parameters = ParametersMap());
    void parseParameters(const ParametersMap& parameters);

    void addLocalMap(int nodeId, std::shared_ptr<const LocalMap> localMap);
    void addLocalMap(int nodeId, const Transform& pose,
        std::shared_ptr<const LocalMap> localMap);

    void cacheCurrentMap();

    void updatePoses(const std::map<int, Transform>& updatedPoses,
        int lastNodeIdToIncludeInMapCache = -1);

    OccupancyGrid getOccupancyGrid() const;
    OccupancyGrid getOccupancyGrid(const MapLimits& roi) const;
    OccupancyGrid getProbOccupancyGrid() const;
    OccupancyGrid getProbOccupancyGrid(const MapLimits& roi) const;
    ColorGrid getColorGrid() const;
    ColorGrid getColorGrid(const MapLimits& roi) const;

    const std::map<int, Node>& nodes() const { return nodes_; }
    std::optional<Transform> getNodePose(int nodeId) const;
    const MapLimits& mapLimits() const { return mapLimits_; }

    void reset();

private:
    bool checkIfCachedMapCanBeUsed(const std::map<int, Transform>& updatedPoses);
    void useCachedMap();
    int tryToUseCachedMap(const std::map<int, Transform>& updatedPoses);

    TransformedLocalMap transformLocalMap(const LocalMap& localMap, const Transform& transform);
    void createOrResizeMap(const MapLimits& newMapLimits);
    void deployLocalMap(int nodeId);
    void deployLocalMap(const Node& node);

    void clear();

    float valueToProbability(int value)
    {
        if (value == 0)
        {
            return 0.5f;
        }
        return 1.0f * (value - 1) / splitNum_;
    }

    int probabilityToVlaue(float probability)
    {
        return std::lround(probability * splitNum_) + 1;
    }

private:
    float cellSize_;
    float missProb_;
    float hitProb_;
    float minClampingProb_;
    float maxClampingProb_;
    float occupancyProbThr_;
    int occupancyThr_;
    int updated_;
    int temporarilyOccupiedCellColorRgb_;
    Color temporarilyOccupiedCellColor_;
    bool showTemporarilyOccupiedCells_;

    std::map<int, Node> nodes_;
    MapLimits mapLimits_;
    MapType map_;
    ColorsType colors_;
    std::list<std::pair<int, int>> temporarilyOccupiedCells_;

    std::map<int, Transform> cachedPoses_;
    MapLimits cachedMapLimits_;
    MapType cachedMap_;
    ColorsType cachedColors_;
    std::list<std::pair<int, int>> cachedTemporarilyOccupiedCells_;

    // value 0 = unknown
    // value 1 = prob 0.0
    // value (splitNum_ + 1) = prob 1.0
    static constexpr int splitNum_ = 1000;
    static constexpr int unknown_ = 0;
    std::vector<int> missUpdates_;
    std::vector<int> hitUpdates_;
    std::vector<std::int8_t> probabilities_;
    std::vector<std::int8_t> probabilitiesThr_;
};

}
