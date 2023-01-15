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
    times_[nodeId] = time;
    occupancyGridMap_->addLocalMap(nodeId, localMap);
}

void TimedOccupancyGridMap::addLocalMap(int nodeId, const Time& time,
    const Transform& pose, std::shared_ptr<const LocalMap> localMap)
{
    UASSERT(times_.count(nodeId) == 0);
    times_[nodeId] = time;
    occupancyGridMap_->addLocalMap(nodeId, pose, localMap);
}

void TimedOccupancyGridMap::addTemporaryLocalMap(const Time& time,
    const Transform& pose, std::shared_ptr<const LocalMap> localMap)
{
    temporaryTimes_.push_back(time);
    if ((int)temporaryTimes_.size() > maxTemporaryLocalMaps())
    {
        temporaryTimes_.pop_front();
    }
    occupancyGridMap_->addTemporaryLocalMap(pose, localMap);
}

void TimedOccupancyGridMap::updatePoses(const Trajectories& trajectories,
    bool autoCaching /* false */)
{
    int lastNodeIdToIncludeInCachedMap = -1;
    const Trajectory* latestTrajectory = trajectories.getLatestTrajectory();

    std::map<int, Transform> updatedPoses;
    for (const auto& idTime: times_)
    {
        int nodeId = idTime.first;
        Time time = idTime.second;
        std::optional<Transform> pose;
        std::optional<Transform> prevPose = getNodePose(nodeId);
        const Trajectory* trajectory;
        std::tie(pose, trajectory) = getPose(trajectories, time, prevPose);
        if (pose.has_value())
        {
            updatedPoses[nodeId] = *pose;
            if (autoCaching && trajectory != latestTrajectory)
            {
                lastNodeIdToIncludeInCachedMap =
                    std::max(lastNodeIdToIncludeInCachedMap, nodeId);
            }
        }
    }

    std::list<Transform> updatedTemporaryPoses;
    int i = 0;
    for (const Time& temporaryTime : temporaryTimes_)
    {
        Transform prevPose = getTemporaryNodePose(i);
        std::optional<Transform> pose =
            getPose(trajectories, temporaryTime, prevPose).first;
        UASSERT(pose.has_value());
        updatedTemporaryPoses.push_back(*pose);
        i++;
    }

    occupancyGridMap_->updatePoses(updatedPoses,
        updatedTemporaryPoses, lastNodeIdToIncludeInCachedMap);

    prevTrajectories_ = trajectories;
}

std::pair<std::optional<Transform>, const TimedOccupancyGridMap::Trajectory*>
TimedOccupancyGridMap::getPose(
    const Trajectories& trajectories, const Time& time,
    std::optional<Transform> prevPose /* std::nullopt */)
{
    if (trajectories.size() == 0)
    {
        return std::make_pair(std::nullopt, nullptr);
    }
    for (const Trajectory& trajectory : trajectories)
    {
        std::optional<Transform> pose = getPose(trajectory, time);
        if (pose.has_value())
        {
            return std::make_pair(std::move(pose), &trajectory);
        }
    }
    if (prevPose.has_value())
    {
        const Trajectory* prevTrajectory = prevTrajectories_.findContinuedTrajectory(time);
        const Trajectory* trajectory = trajectories.findContinuedTrajectory(time);
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
                return std::make_pair(std::move(pose), trajectory);
            }
        }
    }
    return std::make_pair(std::nullopt, nullptr);
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
        *nodeProto.mutable_time() = rtabmap::toProto(times_.at(nodeId));
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
        maxNodeId = std::max(maxNodeId, nodeId);
    }
    reader.close();
    return maxNodeId + 1;
}

}
