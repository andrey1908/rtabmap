#pragma once

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/LocalMapBuilder.h>
#include <rtabmap/core/OccupancyGridBuilder.h>
#include <rtabmap/core/TemporaryOccupancyGridBuilder.h>
#include <rtabmap/core/ObstacleDilation.h>

#include <yaml-cpp/yaml.h>

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
        ObstacleDilation::Parameters obstacleDilationParameters;
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
                parameters.obstacleDilationParameters =
                    ObstacleDilation::Parameters::createParameters(
                        node["ObstacleDilation"]);
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

    void cacheCurrentMap();

    void updatePoses(const std::map<int, Transform>& updatedPoses,
        const std::deque<Transform>& updatedTemporaryPoses,
        int lastNodeIdToIncludeInCachedMap = -1);

    OccupancyGrid getOccupancyGrid() const;
    OccupancyGrid getProbOccupancyGrid() const;
    ColorGrid getColorGrid() const;

    float cellSize() const { return cellSize_; }
    std::pair<float, float> getGridOrigin() const;
    int maxTemporaryLocalMaps() const
        { return temporaryOccupancyGridBuilder_->maxTemporaryLocalMaps(); }
    const std::map<int, Node>& nodes() const { return occupancyGridBuilder_->nodes(); }
    const std::deque<Node>& temporaryNodes() const
        { return temporaryOccupancyGridBuilder_->nodes(); }
    const std::map<int, const std::shared_ptr<const LocalMap>>&
        localMapsWithoutObstacleDilation() const { return localMapsWithoutObstacleDilation_; }
    const cv::Mat& lastDilatedSemantic() const
        { return localMapBuilder_->lastDilatedSemantic(); }

    void reset();

private:
    float cellSize_;

    std::unique_ptr<LocalMapBuilder> localMapBuilder_;
    std::unique_ptr<ObstacleDilation> obstacleDilation_;
    std::unique_ptr<OccupancyGridBuilder> occupancyGridBuilder_;
    std::unique_ptr<TemporaryOccupancyGridBuilder> temporaryOccupancyGridBuilder_;

    std::map<int, const std::shared_ptr<const LocalMap>> localMapsWithoutObstacleDilation_;
};

}
