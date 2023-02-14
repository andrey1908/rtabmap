#pragma once

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/Time.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Trajectory.h>
#include <rtabmap/core/OccupancyGridMap.h>

#include <yaml-cpp/yaml.h>

#include <vector>
#include <set>
#include <map>
#include <optional>
#include <utility>
#include <memory>

namespace rtabmap {

class TimedOccupancyGridMap
{
public:
    struct Parameters
    {
        float maxInterpolationTimeError = 0.06f;
        float guaranteedInterpolationTimeWindow = 1.0f;

        OccupancyGridMap::Parameters occupancyGridMapParameters;

        static Parameters createParameters(const YAML::Node& node)
        {
            UASSERT(node.IsMap());
            Parameters parameters;
            if (node["MaxInterpolationTimeError"])
            {
                parameters.maxInterpolationTimeError =
                    node["MaxInterpolationTimeError"].as<float>();
            }
            if (node["GuaranteedInterpolationTimeWindow"])
            {
                parameters.guaranteedInterpolationTimeWindow =
                    node["GuaranteedInterpolationTimeWindow"].as<float>();
            }
            if (node["OccupancyGridMap"])
            {
                parameters.occupancyGridMapParameters =
                    OccupancyGridMap::Parameters::createParameters(
                        node["OccupancyGridMap"]);
            }
            return parameters;
        }
    };

public:
    TimedOccupancyGridMap(const Parameters& parameters);
    void parseParameters(const Parameters& parameters);

    std::shared_ptr<LocalMap> createLocalMap(const SensorData& sensorData,
        const Time& time, const Transform& fromUpdatedPose) const;

    int addLocalMap(const std::shared_ptr<const LocalMap>& localMap);
    int addLocalMap(const Transform& pose,
        const std::shared_ptr<const LocalMap>& localMap);
    bool addTemporaryLocalMap(const Transform& pose,
        const std::shared_ptr<const LocalMap>& localMap);

    int addSensorData(const SensorData& sensorData, const Time& time,
        const Transform& fromUpdatedPose);
    int addSensorData(const SensorData& sensorData, const Time& time,
        const Transform& pose, const Transform& fromUpdatedPose);
    bool addTemporarySensorData(const SensorData& sensorData, const Time& time,
        const Transform& pose, const Transform& fromUpdatedPose);

    void transformMap(const Transform& transform)
        { occupancyGridMap_->transformMap(transform); }

    void updatePoses(const Trajectories& trajectories);

    OccupancyGrid getOccupancyGrid(int index) const
        { return occupancyGridMap_->getOccupancyGrid(index); }
    OccupancyGrid getProbOccupancyGrid(int index) const
        { return occupancyGridMap_->getProbOccupancyGrid(index); }
    ColorGrid getColorGrid(int index) const
        { return occupancyGridMap_->getColorGrid(index); }

    float cellSize() const { return occupancyGridMap_->cellSize(); }
    std::pair<float, float> getGridOrigin(int index) const
        { return occupancyGridMap_->getGridOrigin(index); }
    int maxTemporaryLocalMaps(int index) const
        { return occupancyGridMap_->maxTemporaryLocalMaps(index); }
    const std::map<int, Node>& nodes(int index) const { return occupancyGridMap_->nodes(index); }
    const std::deque<Node>& temporaryNodes(int index) const
        { return occupancyGridMap_->temporaryNodes(index); }
    const std::map<int, const std::shared_ptr<const LocalMap>>&
        localMapsWithoutObstacleDilation() const
            { return occupancyGridMap_->localMapsWithoutObstacleDilation(); }
    const cv::Mat& lastDilatedSemantic() const
        { return occupancyGridMap_->lastDilatedSemantic(); }
    int numBuilders() const { return occupancyGridMap_->numBuilders(); };

    void reset();

    void save(const std::string& file);
    void load(const std::string& file);

private:
    std::pair<std::optional<Transform>, bool /* if pose was extrapolated */> getPose(
        const Trajectories& trajectories, const Time& time, bool canExtrapolate,
        const std::optional<Transform>& prevPose = std::nullopt);
    std::pair<std::optional<Transform>, bool /* if trajectory contains time */> getPose(
        const Trajectory& trajectory, const Time& time);

private:
    double maxInterpolationTimeError_;
    double guaranteedInterpolationTimeWindow_;

    std::map<int, bool> canExtrapolate_;
    std::deque<bool> temporaryCanExtrapolate_;
    std::unique_ptr<OccupancyGridMap> occupancyGridMap_;

    Trajectories prevTrajectories_;
};

}
