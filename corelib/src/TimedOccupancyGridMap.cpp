#include <rtabmap/core/TimedOccupancyGridMap.h>
#include <rtabmap/core/Serialization.h>

#include <rtabmap/proto/OccupancyGridMap.pb.h>

#include "time_measurer/time_measurer.h"

namespace rtabmap {

TimedOccupancyGridMap::TimedOccupancyGridMap(
    const ParametersMap& parameters /* ParametersMap() */) :
    maxInterpolationTimeError_(Parameters::defaultTimedGridMaxInterpolationTimeError()),
    guaranteedInterpolationTimeWindow_(
        Parameters::defaultTimedGridGuaranteedInterpolationTimeWindow())
{
    parseParameters(parameters);
}

void TimedOccupancyGridMap::parseParameters(const ParametersMap& parameters)
{
    Parameters::parse(parameters, Parameters::kTimedGridMaxInterpolationTimeError(),
        maxInterpolationTimeError_);
    Parameters::parse(
        parameters, Parameters::kTimedGridGuaranteedInterpolationTimeWindow(),
        guaranteedInterpolationTimeWindow_);

    occupancyGridMap_ = std::make_unique<OccupancyGridMap>(parameters);
}

void TimedOccupancyGridMap::addLocalMap(int nodeId, const Time& time,
    std::shared_ptr<const LocalMap> localMap)
{
    UASSERT(times_.count(nodeId) == 0);
    times_.emplace(nodeId, NodeTime{time, true});
    occupancyGridMap_->addLocalMap(nodeId, localMap);
}

void TimedOccupancyGridMap::addLocalMap(int nodeId, const Time& time,
    const Transform& pose, std::shared_ptr<const LocalMap> localMap)
{
    UASSERT(times_.count(nodeId) == 0);
    times_.emplace(nodeId, NodeTime{time, true});
    occupancyGridMap_->addLocalMap(nodeId, pose, localMap);
}

void TimedOccupancyGridMap::addTemporaryLocalMap(const Time& time,
    const Transform& pose, std::shared_ptr<const LocalMap> localMap)
{
    temporaryTimes_.emplace_back(NodeTime{time, true});
    if ((int)temporaryTimes_.size() > maxTemporaryLocalMaps())
    {
        temporaryTimes_.pop_front();
    }
    occupancyGridMap_->addTemporaryLocalMap(pose, localMap);
}

void TimedOccupancyGridMap::updatePoses(const Trajectories& trajectories)
{
    std::map<int, Transform> updatedPoses;
    for (auto& idTime: times_)
    {
        int nodeId = idTime.first;
        NodeTime& time = idTime.second;
        std::optional<Transform> prevPose = getNodePose(nodeId);
        std::optional<Transform> pose;
        bool extrapolated;
        std::tie(pose, extrapolated) = getPose(trajectories, time, prevPose);
        time.canExtrapolate = extrapolated;
        if (pose.has_value())
        {
            updatedPoses[nodeId] = *pose;
        }
    }

    std::list<Transform> updatedTemporaryPoses;
    int i = 0;
    for (NodeTime& temporaryTime : temporaryTimes_)
    {
        Transform prevPose = getTemporaryNodePose(i);
        std::optional<Transform> pose;
        bool extrapolated;
        std::tie(pose, extrapolated) = getPose(trajectories, temporaryTime, prevPose);
        UASSERT(pose.has_value());
        temporaryTime.canExtrapolate = extrapolated;
        updatedTemporaryPoses.push_back(*pose);
        i++;
    }

    int lastNodeIdToIncludeInCachedMap = -1;
    const Trajectory* latestTrajectory = trajectories.getLatestTrajectory();
    if (latestTrajectory != nullptr)
    {
        for (const auto& idTime: times_)
        {
            int nodeId = idTime.first;
            const NodeTime& time = idTime.second;
            if (latestTrajectory->includesTime(time.time))
            {
                break;
            }
            if (updatedPoses.count(nodeId))
            {
                lastNodeIdToIncludeInCachedMap =
                    std::max(lastNodeIdToIncludeInCachedMap, nodeId);
            }
        }
    }

    prevTrajectories_ = trajectories;
    occupancyGridMap_->updatePoses(updatedPoses, updatedTemporaryPoses,
        lastNodeIdToIncludeInCachedMap);
}

std::pair<std::optional<Transform>, bool /* if pose was extrapolated */>
TimedOccupancyGridMap::getPose(
    const Trajectories& trajectories, const NodeTime& time,
    std::optional<Transform> prevPose /* std::nullopt */)
{
    if (trajectories.size() == 0)
    {
        return std::make_pair(std::nullopt, false);
    }
    for (const Trajectory& trajectory : trajectories)
    {
        std::optional<Transform> pose = getPose(trajectory, time.time);
        if (pose.has_value())
        {
            return std::make_pair(std::move(pose), false);
        }
    }
    if (time.canExtrapolate && prevPose.has_value())
    {
        const Trajectory* prevTrajectory =
            prevTrajectories_.findContinuedTrajectory(time.time);
        const Trajectory* trajectory =
            trajectories.findContinuedTrajectory(time.time);
        if (prevTrajectory && trajectory)
        {
            Time commonEndTime = std::min(prevTrajectory->maxTime(), trajectory->maxTime());
            std::optional<Transform> prevTrajectoryEnd =
                getPose(*prevTrajectory, commonEndTime);
            std::optional<Transform> trajectoryEnd =
                getPose(*trajectory, commonEndTime);
            if (prevTrajectoryEnd.has_value() && trajectoryEnd.has_value())
            {
                Transform shift = prevTrajectoryEnd->inverse() * (*trajectoryEnd);
                Transform pose = shift * (*prevPose);
                return std::make_pair(std::move(pose), true);
            }
        }
    }
    return std::make_pair(std::nullopt, false);
}

std::optional<Transform> TimedOccupancyGridMap::getPose(
    const Trajectory& trajectory, const Time& time)
{
    const auto& bounds = trajectory.getBounds(time);
    if (bounds.first == trajectory.end())
    {
        return std::nullopt;
    }
    if (bounds.first->time == time)
    {
        return bounds.first->pose;
    }
    UASSERT(bounds.second != trajectory.end());
    if (std::min(
            std::abs(time.toSec() - bounds.first->time.toSec()),
            std::abs(time.toSec() - bounds.second->time.toSec())) >
                maxInterpolationTimeError_ &&
        trajectory.maxTime().toSec() - time.toSec() >
            guaranteedInterpolationTimeWindow_)
    {
        return std::nullopt;
    }
    float t = (time.toSec() - bounds.first->time.toSec()) /
        (bounds.second->time.toSec() - bounds.first->time.toSec());
    UASSERT(t > 0.0 && t < 1.0);
    return bounds.first->pose.interpolate(t, bounds.second->pose);
}

void TimedOccupancyGridMap::reset()
{
    times_.clear();
    temporaryTimes_.clear();
    occupancyGridMap_->reset();
}

void TimedOccupancyGridMap::save(const std::string& file)
{
    MEASURE_BLOCK_TIME(TimedOccupancyGridMap__save);
    Serialization writer(file);
    proto::OccupancyGridMap::MetaData metaData;
    metaData.set_cell_size(cellSize());
    writer.write(metaData);

    const auto& localMaps = localMapsWithoutObstacleDilation();
    for (const auto& [nodeId, node] : nodes())
    {
        UASSERT(times_.count(nodeId));
        UASSERT(localMaps.count(nodeId));
        proto::OccupancyGridMap::Node nodeProto;
        nodeProto.set_node_id(nodeId);
        *nodeProto.mutable_time() = rtabmap::toProto(times_.at(nodeId).time);
        if (node.transformedLocalMap.has_value())
        {
            *nodeProto.mutable_pose() =
                rtabmap::toProto(node.transformedLocalMap->pose);
        }
        *nodeProto.mutable_local_map() = rtabmap::toProto(*localMaps.at(nodeId));
        writer.write(nodeProto);
    }
    writer.close();
}

int TimedOccupancyGridMap::load(const std::string& file)
{
    MEASURE_BLOCK_TIME(TimedOccupancyGridMap__load);
    reset();
    Deserialization reader(file);
    UASSERT(cellSize() == reader.getMetaData().cell_size());
    std::optional<proto::OccupancyGridMap::Node> nodeProto;
    int maxNodeId = -1;
    while (nodeProto = reader.readNode())
    {
        UASSERT(nodeProto->has_time());
        int nodeId = nodeProto->node_id();
        Time time = rtabmap::fromProto(nodeProto->time());
        std::shared_ptr<LocalMap> localMap =
            rtabmap::fromProto(nodeProto->local_map());
        if (nodeProto->has_pose())
        {
            Transform pose = rtabmap::fromProto(nodeProto->pose());
            addLocalMap(nodeId, time, pose, localMap);
        }
        else
        {
            addLocalMap(nodeId, time, localMap);
        }
        times_.at(nodeId).canExtrapolate = false;
        maxNodeId = std::max(maxNodeId, nodeId);
    }
    reader.close();
    return maxNodeId + 1;
}

}
