#include <rtabmap/core/TimedOccupancyGridMap.h>
#include <rtabmap/core/Serialization.h>

#include <rtabmap/proto/OccupancyGridMap.pb.h>

#include "time_measurer/time_measurer.h"

namespace rtabmap {

TimedOccupancyGridMap::TimedOccupancyGridMap(
    const ParametersMap& parameters /* ParametersMap() */) :
    maxInterpolationTimeError_(Parameters::defaultTimedGridMaxInterpolationTimeError()),
    maxExtrapolationTime_(Parameters::defaultTimedGridMaxExtrapolationTime())
{
    parseParameters(parameters);
}

void TimedOccupancyGridMap::parseParameters(const ParametersMap& parameters)
{
    Parameters::parse(parameters, Parameters::kTimedGridMaxInterpolationTimeError(),
        maxInterpolationTimeError_);
    Parameters::parse(parameters, Parameters::kTimedGridMaxExtrapolationTime(),
        maxExtrapolationTime_);

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
        const Trajectory* trajectory;
        std::tie(pose, trajectory) = getPose(trajectories, time);
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
    for (const Time& temporaryTime : temporaryTimes_)
    {
        std::optional<Transform> pose = getPose(trajectories, temporaryTime).first;
        UASSERT(pose.has_value());
        updatedTemporaryPoses.push_back(*pose);
    }

    occupancyGridMap_->updatePoses(updatedPoses,
        updatedTemporaryPoses, lastNodeIdToIncludeInCachedMap);
}

std::pair<std::optional<Transform>, const TimedOccupancyGridMap::Trajectory*>
TimedOccupancyGridMap::getPose(
    const Trajectories& trajectories, const Time& time)
{
    if (trajectories.size() == 0)
    {
        return std::make_pair(std::nullopt, nullptr);
    }

    const Trajectory* closestTrajectory = nullptr;
    double minDist = std::numeric_limits<double>::max();
    for (const Trajectory& trajectory : trajectories)
    {
        if (trajectory.maxTime() < time)
        {
            double dist = time.toSec() - trajectory.maxTime().toSec();
            if (dist < minDist)
            {
                closestTrajectory = &trajectory;
                minDist = dist;
            }
            continue;
        }
        const auto& bounds = trajectory.getBounds(time);
        if (bounds.first == trajectory.end())
        {
            continue;
        }
        if (bounds.first->time == time)
        {
            return std::make_pair(bounds.first->pose, &trajectory);
        }
        UASSERT(bounds.second != trajectory.end());
        if (std::min(
                std::abs(time.toSec() - bounds.first->time.toSec()),
                std::abs(time.toSec() - bounds.second->time.toSec())) >
            maxInterpolationTimeError_)
        {
            continue;
        }
        float t = (time.toSec() - bounds.first->time.toSec()) /
            (bounds.second->time.toSec() - bounds.first->time.toSec());
        UASSERT(t > 0.0 && t < 1.0);
        return std::make_pair(
            bounds.first->pose.interpolate(t, bounds.second->pose),
            &trajectory);
    }
    if (closestTrajectory != nullptr && minDist <= maxExtrapolationTime_)
    {
        if (closestTrajectory->size() == 1)
        {
            return std::make_pair(closestTrajectory->begin()->pose,
                closestTrajectory);
        }
        auto second = std::prev(closestTrajectory->end());
        auto first = std::prev(second);
        float t = (time.toSec() - first->time.toSec()) /
            (second->time.toSec() - first->time.toSec());
        UASSERT(t > 1.0);
        return std::make_pair(first->pose.interpolate(t, second->pose),
            closestTrajectory);
    }
    return std::make_pair(std::nullopt, nullptr);
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
