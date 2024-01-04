#include <rtabmap/core/PosesApproximation.h>

#include <kas_utils/time_measurer.h>

namespace rtabmap {

void PosesApproximation::addLocalPose(const Time& time, const Transform& localPose)
{
    localPoses_.addPose(time, localPose);
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
        std::optional<Transform> pose = approximate(trajectories, time, nodeId);
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

std::optional<Transform> PosesApproximation::approximate(
    const Trajectories& trajectories, const Time& time, int nodeId /* for logs */) const
{
    for (const Trajectory& trajectory : trajectories)
    {
        if (!trajectory.containsTime(time))
        {
            continue;
        }

        Trajectory::Bounds bounds = trajectory.getBounds(time);
        if (bounds.second == trajectory.end())
        {
            UASSERT(bounds.first->time == time);
            return bounds.first->pose;
        }

        const Time& time1 = bounds.first->time;
        const Transform& globalPose1 = bounds.first->pose;
        const Time& time2 = bounds.second->time;
        const Transform globalPose2 = bounds.second->pose;

        std::optional<Transform> localPosePtr = localPoses_.getPose(time);
        std::optional<Transform> localPose1Ptr = localPoses_.getPose(time1);
        std::optional<Transform> localPose2Ptr = localPoses_.getPose(time2);

        if (!localPosePtr || !localPose1Ptr || !localPose2Ptr)
        {
            // UWARN("Could not approximate pose for node %d.", nodeId);
            return std::nullopt;
        }

        const Transform& localPose = *localPosePtr;
        const Transform& localPose1 = *localPose1Ptr;
        const Transform& localPose2 = *localPose2Ptr;

        Transform diff1 = localPose1.inverse() * localPose;
        Transform diff2 = localPose2.inverse() * localPose;
        Transform approx1 = globalPose1 * diff1;
        Transform approx2 = globalPose2 * diff2;

        float t = (time.toSec() - time1.toSec()) / (time2.toSec() - time1.toSec());
        Transform approx = approx1.interpolate(t, approx2);
        return approx;
    }
    return std::nullopt;
}

void PosesApproximation::reset()
{
    localPoses_.clear();
    times_.clear();
}

}