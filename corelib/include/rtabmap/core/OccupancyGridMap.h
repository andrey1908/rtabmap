#pragma once

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/LocalMapBuilder.h>
#include <rtabmap/core/OccupancyGridBuilder.h>
#include <rtabmap/core/TemporaryOccupancyGridBuilder.h>
#include <rtabmap/core/ObstacleDilation.h>

#include <yaml-cpp/yaml.h>

#include <vector>
#include <memory>
#include <optional>
#include <climits>

namespace rtabmap {

class OccupancyGridMap
{
public:
    struct Parameters
    {
        float cellSize = 0.1f;
        LocalMapBuilder::Parameters localMapBuilderParameters;
        std::vector<ObstacleDilation::Parameters> obstacleDilationsParameters;
        OccupancyGridBuilder::Parameters occupancyGridBuilderParameters;
        TemporaryOccupancyGridBuilder::Parameters
            temporaryOccupancyGridBuilderParameters;

        static Parameters createParameters(const YAML::Node& node)
        {
            UASSERT(node.IsMap());
            Parameters parameters;
            if (node["CellSize"])
            {
                parameters.cellSize = node["CellSize"].as<float>();
            }
            if (node["LocalMapBuilder"])
            {
                parameters.localMapBuilderParameters =
                    LocalMapBuilder::Parameters::createParameters(
                        node["LocalMapBuilder"]);
            }
            if (node["ObstacleDilation"])
            {
                UASSERT(node["ObstacleDilation"].IsMap() ||
                    node["ObstacleDilation"].IsSequence());
                if (node["ObstacleDilation"].IsMap())
                {
                    parameters.obstacleDilationsParameters.push_back(
                        ObstacleDilation::Parameters::createParameters(
                            node["ObstacleDilation"]));
                }
                if (node["ObstacleDilation"].IsSequence())
                {
                    for (const YAML::Node& obstacleDilationNode : node["ObstacleDilation"])
                    {
                        parameters.obstacleDilationsParameters.push_back(
                            ObstacleDilation::Parameters::createParameters(
                                obstacleDilationNode));
                    }
                }
            }
            else
            {
                ObstacleDilation::Parameters obstacleDilationParameters;
                obstacleDilationParameters.dilationSize = 0.0f;
                parameters.obstacleDilationsParameters.push_back(
                    obstacleDilationParameters);
            }
            if (node["OccupancyGridBuilder"])
            {
                parameters.occupancyGridBuilderParameters =
                    OccupancyGridBuilder::Parameters::createParameters(
                        node["OccupancyGridBuilder"]);
            }
            if (node["TemporaryOccupancyGridBuilder"])
            {
                parameters.temporaryOccupancyGridBuilderParameters =
                    TemporaryOccupancyGridBuilder::Parameters::createParameters(
                        node["TemporaryOccupancyGridBuilder"]);
            }
            return parameters;
        }
    };

public:
    OccupancyGridMap(const Parameters& parameters);
    void parseParameters(const Parameters& parameters);

    std::shared_ptr<LocalMap> createLocalMap(const Signature& signature,
        const Transform& fromUpdatedPose = Transform::getIdentity()) const;

    void addLocalMap(int nodeId,
        std::shared_ptr<const LocalMap> localMap);
    void addLocalMap(int nodeId, const Transform& pose,
        std::shared_ptr<const LocalMap> localMap);
    void addTemporaryLocalMap(const Transform& pose,
        std::shared_ptr<const LocalMap> localMap);

    void updatePoses(const std::map<int, Transform>& updatedPoses,
        const std::deque<Transform>& updatedTemporaryPoses,
        int lastNodeIdToIncludeInCachedMap = -1);

    OccupancyGrid getOccupancyGrid(int index) const;
    OccupancyGrid getProbOccupancyGrid(int index) const;
    ColorGrid getColorGrid(int index) const;

    float cellSize() const { return cellSize_; }
    std::pair<float, float> getGridOrigin(int index) const;
    int maxTemporaryLocalMaps(int index) const
        { return temporaryOccupancyGridBuilders_[index]->maxTemporaryLocalMaps(); }
    const std::map<int, Node>& nodes(int index) const { return occupancyGridBuilders_[index]->nodes(); }
    const std::deque<Node>& temporaryNodes(int index) const
        { return temporaryOccupancyGridBuilders_[index]->nodes(); }
    const std::map<int, const std::shared_ptr<const LocalMap>>&
        localMapsWithoutObstacleDilation() const { return localMapsWithoutObstacleDilation_; }
    const cv::Mat& lastDilatedSemantic() const
        { return localMapBuilder_->lastDilatedSemantic(); }
    int numBuilders() const { return numBuilders_; }

    void reset();

private:
    float cellSize_;

    std::unique_ptr<LocalMapBuilder> localMapBuilder_;

    int numBuilders_;
    std::vector<std::unique_ptr<ObstacleDilation>> obstacleDilations_;
    std::vector<std::unique_ptr<OccupancyGridBuilder>> occupancyGridBuilders_;
    std::vector<std::unique_ptr<TemporaryOccupancyGridBuilder>>
        temporaryOccupancyGridBuilders_;

    std::map<int, const std::shared_ptr<const LocalMap>> localMapsWithoutObstacleDilation_;
};

}