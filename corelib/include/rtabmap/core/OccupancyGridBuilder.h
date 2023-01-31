#pragma once

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/LocalMapBuilder.h>
#include <rtabmap/core/Grid.h>
#include <rtabmap/core/MapLimits.h>
#include <rtabmap/core/Color.h>
#include <rtabmap/core/Node.h>

#include <yaml-cpp/yaml.h>

#include <list>
#include <map>
#include <utility>
#include <optional>
#include <Eigen/Core>

namespace rtabmap {

class OccupancyGridBuilder
{
public:
    struct Parameters
    {
        float cellSize = 0.1f;
        float missProb = 0.4f;
        float hitProb = 0.7f;
        float minClampingProb = 0.1192f;
        float maxClampingProb = 0.971f;
        float occupancyProbThr = 0.5f;
        int temporarilyOccupiedCellColorRgb = -1;
        bool showTemporarilyOccupiedCells = true;

        static Parameters createParameters(const YAML::Node& node)
        {
            UASSERT(node.IsMap());
            Parameters parameters;
            if (node["CellSize"])
            {
                parameters.cellSize = node["CellSize"].as<float>();
            }
            if (node["MissProb"])
            {
                parameters.missProb = node["MissProb"].as<float>();
            }
            if (node["HitProb"])
            {
                parameters.hitProb = node["HitProb"].as<float>();
            }
            if (node["MinClampingProb"])
            {
                parameters.minClampingProb = node["MinClampingProb"].as<float>();
            }
            if (node["MaxClampingProb"])
            {
                parameters.maxClampingProb = node["MaxClampingProb"].as<float>();
            }
            if (node["OccupancyProbThr"])
            {
                parameters.occupancyProbThr = node["OccupancyProbThr"].as<float>();
            }
            if (node["TemporarilyOccupiedCellColorRgb"])
            {
                parameters.temporarilyOccupiedCellColorRgb =
                    node["TemporarilyOccupiedCellColorRgb"].as<int>();
            }
            if (node["ShowTemporarilyOccupiedCells"])
            {
                parameters.showTemporarilyOccupiedCells =
                    node["ShowTemporarilyOccupiedCells"].as<bool>();
            }
            return parameters;
        }
    };

private:
    using MapType = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    using ColorsType = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

public:
    OccupancyGridBuilder(const Parameters& parameters);
    void parseParameters(const Parameters& parameters);

    void addLocalMap(int nodeId, std::shared_ptr<const LocalMap> localMap);
    void addLocalMap(int nodeId, const Transform& pose,
        std::shared_ptr<const LocalMap> localMap);

    void cacheCurrentMap();

    void updatePoses(const std::map<int, Transform>& updatedPoses,
        int lastNodeIdToIncludeInMapCache = -1);

    OccupancyGrid getOccupancyGrid() const;
    OccupancyGrid getOccupancyGrid(const MapLimitsI& roi) const;
    OccupancyGrid getProbOccupancyGrid() const;
    OccupancyGrid getProbOccupancyGrid(const MapLimitsI& roi) const;
    ColorGrid getColorGrid() const;
    ColorGrid getColorGrid(const MapLimitsI& roi) const;

    const std::map<int, Node>& nodes() const { return nodes_; }
    const MapLimitsI& mapLimits() const { return mapLimits_; }

    void reset();

private:
    bool checkIfCachedMapCanBeUsed(const std::map<int, Transform>& newPoses);
    void useCachedMap();
    int tryToUseCachedMap(const std::map<int, Transform>& newPoses);

    TransformedLocalMap transformLocalMap(const LocalMap& localMap,
        const Transform& transform);
    void createOrResizeMap(const MapLimitsI& newMapLimits);
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
    MapLimitsI mapLimits_;
    MapType map_;
    ColorsType colors_;
    std::list<std::pair<int, int>> temporarilyOccupiedCells_;

    std::map<int, Transform> cachedPoses_;
    MapLimitsI cachedMapLimits_;
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
