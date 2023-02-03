#pragma once

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/LocalMapBuilder.h>
#include <rtabmap/core/Grid.h>
#include <rtabmap/core/MapLimits.h>
#include <rtabmap/core/Color.h>
#include <rtabmap/core/Node.h>

#include <yaml-cpp/yaml.h>

#include <deque>
#include <map>
#include <utility>
#include <optional>
#include <Eigen/Core>

namespace rtabmap {

class TemporaryOccupancyGridBuilder
{
public:
    struct Parameters
    {
        float cellSize = 0.1f;
        float temporaryMissProb = 0.4f;
        float temporaryHitProb = 0.7f;
        float temporaryOccupancyProbThr = 0.5f;
        int maxTemporaryLocalMaps = 1;

        static Parameters createParameters(const YAML::Node& node)
        {
            UASSERT(node.IsMap());
            Parameters parameters;
            if (node["CellSize"])
            {
                parameters.cellSize = node["CellSize"].as<float>();
            }
            if (node["TemporaryMissProb"])
            {
                parameters.temporaryMissProb = node["TemporaryMissProb"].as<float>();
            }
            if (node["TemporaryHitProb"])
            {
                parameters.temporaryHitProb = node["TemporaryHitProb"].as<float>();
            }
            if (node["TemporaryOccupancyProbThr"])
            {
                parameters.temporaryOccupancyProbThr =
                    node["TemporaryOccupancyProbThr"].as<float>();
            }
            if (node["MaxTemporaryLocalMaps"])
            {
                parameters.maxTemporaryLocalMaps =
                    node["MaxTemporaryLocalMaps"].as<int>();
            }
            return parameters;
        }
    };

private:
    using CounterType = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    using ColorsType = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

    struct PrecomputedProbabilities
    {
        Eigen::Matrix<std::int8_t, Eigen::Dynamic, Eigen::Dynamic> probabilities;
        Eigen::Matrix<std::int8_t, Eigen::Dynamic, Eigen::Dynamic> probabilitiesThr;
    };

public:
    TemporaryOccupancyGridBuilder(const Parameters& parameters);
    void parseParameters(const Parameters& parameters);

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
    void precomputeProbabilities();

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

    PrecomputedProbabilities precomputedProbabilities_;
};

}
