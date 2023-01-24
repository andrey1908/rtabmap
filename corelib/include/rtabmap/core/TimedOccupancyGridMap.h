#pragma once

#include <rtabmap/core/Time.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Trajectory.h>
#include <rtabmap/core/OccupancyGridMap.h>
#include <rtabmap/utilite/ULogger.h>

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
    TimedOccupancyGridMap(const ParametersMap& parameters = ParametersMap());
    void parseParameters(const ParametersMap& parameters);

    std::shared_ptr<LocalMap> createLocalMap(const Signature& signature, const Time& time,
        const Transform& fromUpdatedPose = Transform::getIdentity()) const;

    void addLocalMap(int nodeId,
        std::shared_ptr<const LocalMap> localMap);
    void addLocalMap(int nodeId, const Transform& pose,
        std::shared_ptr<const LocalMap> localMap);
    void addTemporaryLocalMap(const Transform& pose,
        std::shared_ptr<const LocalMap> localMap);

    void cacheCurrentMap() { occupancyGridMap_->cacheCurrentMap(); }

    void updatePoses(const Trajectories& trajectories);

    OccupancyGrid getOccupancyGrid() const
        { return occupancyGridMap_->getOccupancyGrid(); }
    OccupancyGrid getProbOccupancyGrid() const
        { return occupancyGridMap_->getProbOccupancyGrid(); }
    ColorGrid getColorGrid() const
        { return occupancyGridMap_->getColorGrid(); }

    float cellSize() const { return occupancyGridMap_->cellSize(); }
    std::pair<float, float> getGridOrigin() const
        { return occupancyGridMap_->getGridOrigin(); }
    int maxTemporaryLocalMaps() const
        { return occupancyGridMap_->maxTemporaryLocalMaps(); }
    const std::map<int, Node>& nodes() const { return occupancyGridMap_->nodes(); }
    const std::deque<Node>& temporaryNodes() const
        { return occupancyGridMap_->temporaryNodes(); }
    const std::map<int, const std::shared_ptr<const LocalMap>>&
        localMapsWithoutObstacleDilation() const
            { return occupancyGridMap_->localMapsWithoutObstacleDilation(); }
    const cv::Mat& lastDilatedSemantic() const
        { return occupancyGridMap_->lastDilatedSemantic(); }

    void reset();

    void save(const std::string& file);
    int load(const std::string& file);

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
