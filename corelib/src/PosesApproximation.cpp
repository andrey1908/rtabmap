#include <rtabmap/core/PosesApproximation.h>

#include <kas_utils/time_measurer.h>

namespace rtabmap {

PosesApproximation::PosesApproximation(const Parameters& parameters)
{
    parseParameters(parameters);
}

void PosesApproximation::parseParameters(const Parameters& parameters)
{
    maxInterpolationTimeError_ = parameters.maxInterpolationTimeError;
}

void PosesApproximation::addTime(int nodeId, const Time& time)
{
    times_[nodeId] = time;
}

void PosesApproximation::removeTimes(const std::vector<int>& nodeIdsToRemove)
{
    for (int nodeIdToRemove : nodeIdsToRemove)
    {
        auto it = times_.find(nodeIdToRemove);
        UASSERT(it != times_.end());
        times_.erase(it);
    }
}

std::pair<std::map<int, Transform>, int> PosesApproximation::approximatePoses(
    const Trajectories& trajectories) const
{
    int lastNodeIdToIncludeInCachedMap = -1;
    std::map<int, Transform> poses;
    for (const auto& [nodeId, time] : times_)
    {
        std::optional<Transform> pose = interpolate(trajectories, time);
        if (pose.has_value())
        {
            auto nodeTrajectoryIt = trajectories.findCurrentTrajectory(time);
            if (nodeTrajectoryIt != trajectories.end() &&
                nodeTrajectoryIt != std::prev(trajectories.end()))
            {
                lastNodeIdToIncludeInCachedMap = nodeId;
            }
            poses[nodeId] = *pose;
        }
    }
    return std::make_pair(std::move(poses), lastNodeIdToIncludeInCachedMap);
}

std::optional<Transform> PosesApproximation::interpolate(
    const Trajectories& trajectories, const Time& time) const
{
    if (trajectories.empty())
    {
        return std::nullopt;
    }
    for (const Trajectory& trajectory : trajectories)
    {
        std::optional<Transform> pose = trajectory.interpolate(time, maxInterpolationTimeError_);
        if (pose.has_value())
        {
            return pose;
        }
    }
    return std::nullopt;
}

void PosesApproximation::reset()
{
    times_.clear();
}

}