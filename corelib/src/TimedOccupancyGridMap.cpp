#include <rtabmap/core/TimedOccupancyGridMap.h>
#include <rtabmap/core/Serialization.h>

#include <rtabmap/proto/OccupancyGridMap.pb.h>

#include "time_measurer/time_measurer.h"

namespace rtabmap {

TimedOccupancyGridMap::TimedOccupancyGridMap(const Parameters& parameters)
{
    parseParameters(parameters);
}

void TimedOccupancyGridMap::parseParameters(const Parameters& parameters)
{
    maxInterpolationTimeError_ = parameters.maxInterpolationTimeError;
    guaranteedInterpolationTimeWindow_ = parameters.guaranteedInterpolationTimeWindow;

    occupancyGridMap_ =
        std::make_unique<OccupancyGridMap>(parameters.occupancyGridMapParameters);
}

std::shared_ptr<LocalMap> TimedOccupancyGridMap::createLocalMap(
    const Signature& signature, const Time& time,
    const Transform& fromUpdatedPose /* Transform::getIdentity() */) const
{
    std::shared_ptr<LocalMap> localMap =
        occupancyGridMap_->createLocalMap(signature, fromUpdatedPose);
    localMap->setTime(time);
    return localMap;
}

void TimedOccupancyGridMap::addLocalMap(int nodeId,
    std::shared_ptr<const LocalMap> localMap)
{
    UASSERT(canExtrapolate_.count(nodeId) == 0);
    canExtrapolate_[nodeId] = true;
    occupancyGridMap_->addLocalMap(nodeId, localMap);
}

void TimedOccupancyGridMap::addLocalMap(int nodeId, const Transform& pose,
    std::shared_ptr<const LocalMap> localMap)
{
    UASSERT(canExtrapolate_.count(nodeId) == 0);
    canExtrapolate_[nodeId] = true;
    occupancyGridMap_->addLocalMap(nodeId, pose, localMap);
}

void TimedOccupancyGridMap::addTemporaryLocalMap(const Transform& pose,
    std::shared_ptr<const LocalMap> localMap)
{
    temporaryCanExtrapolate_.push_back(true);
    if ((int)temporaryCanExtrapolate_.size() > maxTemporaryLocalMaps(0))
    {
        temporaryCanExtrapolate_.pop_front();
    }
    occupancyGridMap_->addTemporaryLocalMap(pose, localMap);
}

void TimedOccupancyGridMap::updatePoses(const Trajectories& trajectories)
{
    std::map<int, Transform> updatedPoses;
    {
        auto canExtrapolateIt = canExtrapolate_.begin();
        const auto& nodesRef = nodes(0);
        auto nodeIt = nodesRef.begin();
        while (canExtrapolateIt != canExtrapolate_.end())
        {
            UASSERT(nodeIt != nodesRef.end());
            int nodeId = canExtrapolateIt->first;
            const Node& node = nodeIt->second;
            const Time& time = node.localMap->time();
            std::optional<Transform> prevPose = std::nullopt;
            if (node.transformedLocalMap.valid())
            {
                prevPose = node.transformedLocalMap.pose();
            }
            std::optional<Transform> pose;
            bool extrapolated;
            std::tie(pose, extrapolated) =
                getPose(trajectories, time, canExtrapolateIt->second, prevPose);
            canExtrapolateIt->second = extrapolated;
            if (pose.has_value())
            {
                updatedPoses[nodeId] = *pose;
            }
            ++canExtrapolateIt;
            ++nodeIt;
        }
        UASSERT(nodeIt == nodesRef.end());
    }

    std::deque<Transform> updatedTemporaryPoses;
    {
        auto canExtrapolateIt = temporaryCanExtrapolate_.begin();
        const auto& nodesRef = temporaryNodes(0);
        auto nodeIt = nodesRef.begin();
        while (canExtrapolateIt != temporaryCanExtrapolate_.end())
        {
            UASSERT(nodeIt != nodesRef.end());
            const Node& node = *nodeIt;
            const Time& time = node.localMap->time();
            const Transform& prevPose = node.transformedLocalMap.pose();
            std::optional<Transform> pose;
            bool extrapolated;
            std::tie(pose, extrapolated) =
                getPose(trajectories, time, *canExtrapolateIt, prevPose);
            UASSERT(pose.has_value());
            *canExtrapolateIt = extrapolated;
            updatedTemporaryPoses.push_back(*pose);
            ++canExtrapolateIt;
            ++nodeIt;
        }
        UASSERT(nodeIt == nodesRef.end());
    }

    int lastNodeIdToIncludeInCachedMap = -1;
    if (trajectories.size())
    {
        auto latestTrajectoryIt = std::prev(trajectories.end());
        const auto& nodesRef = nodes(0);
        for (const auto& [nodeId, node] : nodesRef)
        {
            const Time& time = node.localMap->time();
            if (time >= latestTrajectoryIt->minTime())
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
    const Trajectories& trajectories, const Time& time, bool canExtrapolate,
    const std::optional<Transform>& prevPose /* std::nullopt */)
{
    if (trajectories.size() == 0)
    {
        return std::make_pair(std::nullopt, false);
    }
    for (const Trajectory& trajectory : trajectories)
    {
        std::optional<Transform> pose;
        bool containsTime;
        std::tie(pose, containsTime) = getPose(trajectory, time);
        if (pose.has_value())
        {
            return std::make_pair(std::move(pose), false);
        }
        else if (containsTime)
        {
            return std::make_pair(std::nullopt, false);
        }
    }
    if (canExtrapolate && prevPose.has_value())
    {
        auto prevTrajectoryIt =
            prevTrajectories_.findContinuedTrajectory(time);
        auto trajectoryIt =
            trajectories.findContinuedTrajectory(time);
        if (prevTrajectoryIt != prevTrajectories_.end() &&
            trajectoryIt != trajectories.end())
        {
            Time commonEndTime =
                std::min(prevTrajectoryIt->maxTime(), trajectoryIt->maxTime());
            std::optional<Transform> prevTrajectoryEnd =
                getPose(*prevTrajectoryIt, commonEndTime).first;
            std::optional<Transform> trajectoryEnd =
                getPose(*trajectoryIt, commonEndTime).first;
            if (prevTrajectoryEnd.has_value() && trajectoryEnd.has_value())
            {
                Transform shift = (*trajectoryEnd) * prevTrajectoryEnd->inverse();
                Transform pose = shift * (*prevPose);
                return std::make_pair(std::move(pose), true);
            }
        }
    }
    return std::make_pair(std::nullopt, false);
}

std::pair<std::optional<Transform>, bool /* if trajectory contains time */>
TimedOccupancyGridMap::getPose(
    const Trajectory& trajectory, const Time& time)
{
    const Trajectory::Bounds& bounds = trajectory.getBounds(time);
    if (bounds.first == trajectory.end())
    {
        return std::make_pair(std::nullopt, false);
    }
    if (bounds.first->time == time)
    {
        return std::make_pair(bounds.first->pose, true);
    }
    UASSERT(bounds.second != trajectory.end());
    if (std::min(
            std::abs(time.toSec() - bounds.first->time.toSec()),
            std::abs(time.toSec() - bounds.second->time.toSec())) >
                maxInterpolationTimeError_ &&
        trajectory.maxTime().toSec() - time.toSec() >
            guaranteedInterpolationTimeWindow_)
    {
        return std::make_pair(std::nullopt, true);
    }
    float t = (time.toSec() - bounds.first->time.toSec()) /
        (bounds.second->time.toSec() - bounds.first->time.toSec());
    UASSERT(t > 0.0 && t < 1.0);
    return std::make_pair(
        bounds.first->pose.interpolate(t, bounds.second->pose), true);
}

void TimedOccupancyGridMap::reset()
{
    canExtrapolate_.clear();
    temporaryCanExtrapolate_.clear();
    occupancyGridMap_->reset();
}

void TimedOccupancyGridMap::save(const std::string& file)
{
    MEASURE_BLOCK_TIME(TimedOccupancyGridMap__save);
    Serialization writer(file);
    proto::OccupancyGridMap::MetaData metaData;
    metaData.set_cell_size(cellSize());
    writer.write(metaData);

    const auto& localMapsRef = localMapsWithoutObstacleDilation();
    auto localMapIt = localMapsRef.begin();
    const auto& nodesRef = nodes(0);
    auto nodeIt = nodesRef.begin();
    while (localMapIt != localMapsRef.end())
    {
        UASSERT(nodeIt != nodesRef.end());
        UASSERT(localMapIt->first == nodeIt->first);
        const Node& node = nodeIt->second;
        proto::OccupancyGridMap::Node proto;
        proto.set_node_id(localMapIt->first);
        if (node.transformedLocalMap.valid())
        {
            *proto.mutable_pose() =
                rtabmap::toProto(node.transformedLocalMap.pose());
        }
        *proto.mutable_local_map() = rtabmap::toProto(*(localMapIt->second));
        writer.write(proto);
        ++localMapIt;
        ++nodeIt;
    }
    UASSERT(nodeIt == nodesRef.end());
    writer.close();
}

int TimedOccupancyGridMap::load(const std::string& file)
{
    MEASURE_BLOCK_TIME(TimedOccupancyGridMap__load);
    reset();
    Deserialization reader(file);
    UASSERT(cellSize() == reader.metaData().cell_size());
    std::optional<proto::OccupancyGridMap::Node> proto;
    int maxNodeId = -1;
    while (proto = reader.readNode())
    {
        int nodeId = proto->node_id();
        std::shared_ptr<LocalMap> localMap =
            rtabmap::fromProto(proto->local_map());
        if (proto->has_pose())
        {
            Transform pose = rtabmap::fromProto(proto->pose());
            addLocalMap(nodeId, pose, localMap);
        }
        else
        {
            addLocalMap(nodeId, localMap);
        }
        canExtrapolate_[nodeId] = false;
        maxNodeId = std::max(maxNodeId, nodeId);
    }
    reader.close();
    return maxNodeId + 1;
}

}
