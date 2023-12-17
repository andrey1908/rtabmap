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
    guaranteedInterpolationTimeWindow_ = parameters.guaranteedInterpolationTimeWindow;
}

void PosesApproximation::addNode(int nodeId, const Time& time)
{
    nodeTimes_[nodeId] = time;
}

void PosesApproximation::addNode(int nodeId, const Time& time, const Transform& pose)
{
    UASSERT(time > lastNodeTime_);
    currentTrajectory_.addPose(time, pose);
    nodeTimes_[nodeId] = time;
    lastNodeTime_ = time;
}

void PosesApproximation::addTemporaryNode(const Time& time, const Transform& pose)
{
    UASSERT(time > lastTemporaryNodeTime_);
    currentTrajectory_.addPose(time, pose);
    temporaryNodeTimes_.push_back(time);
    lastTemporaryNodeTime_ = time;
}

void PosesApproximation::removeNodes(const std::vector<int>& nodeIdsToRemove)
{
    for (int nodeIdToRemove : nodeIdsToRemove)
    {
        auto it = nodeTimes_.find(nodeIdToRemove);
        UASSERT(it != nodeTimes_.end());
        nodeTimes_.erase(it);
    }
}

void PosesApproximation::removeFirstTemporaryNode()
{
    UASSERT(temporaryNodeTimes_.size());
    temporaryNodeTimes_.pop_front();
}

void PosesApproximation::transformCurrentTrajectory(const Transform& transform)
{
    Transform transform3DoF = transform.to3DoF();
    Trajectory transformedCurrentTrajectory;
    for (const auto& timedPose : currentTrajectory_)
    {
        const Transform& pose = timedPose.pose;
        Transform transformedPose = transform3DoF * pose;
        transformedCurrentTrajectory.addPose(timedPose.time, transformedPose);
    }
    currentTrajectory_ = std::move(transformedCurrentTrajectory);
}

PosesApproximation::ApproximatedPoses PosesApproximation::approximatePoses(
    const Trajectories& trajectories)
{
    const Trajectory* activeTrajectoryPtr = nullptr;
    if (currentTrajectory_.size())
    {
        auto activeTrajectoryIt = trajectories.findCurrentOrPreviousTrajectory(
            currentTrajectory_.maxTime());
        if (activeTrajectoryIt != trajectories.end())
        {
            activeTrajectoryPtr = &(*activeTrajectoryIt);
        }
    }
    std::optional<Transform> extrapolationShift = std::nullopt;
    if (activeTrajectoryPtr)
    {
        Time commonTime = std::min(
            currentTrajectory_.maxTime(), activeTrajectoryPtr->maxTime());
        std::optional<Transform> currentPose = currentTrajectory_.interpolate(commonTime);
        std::optional<Transform> updatedPose = activeTrajectoryPtr->interpolate(commonTime);
        if (currentPose.has_value() && updatedPose.has_value())
        {
            extrapolationShift = (*updatedPose) * currentPose->inverse();
        }
    }

    if (!extrapolationShift.has_value())
    {
        currentTrajectory_.clear();
        resetTemporary();
    }

    Time minUpdatedCurrentTrajectoryTime = Time::max();
    if (currentTrajectory_.size())
    {
        minUpdatedCurrentTrajectoryTime =
            Time(currentTrajectory_.maxTime().toSec() - guaranteedInterpolationTimeWindow_);
        if (activeTrajectoryPtr)
        {
            minUpdatedCurrentTrajectoryTime = std::min(
                minUpdatedCurrentTrajectoryTime,
                activeTrajectoryPtr->maxTime());
        }
        if (temporaryNodeTimes_.size())
        {
            minUpdatedCurrentTrajectoryTime = std::min(
                minUpdatedCurrentTrajectoryTime,
                temporaryNodeTimes_.front());
        }
    }
    currentTrajectory_.trim(minUpdatedCurrentTrajectoryTime);

    Trajectory updatedCurrentTrajectory;
    int lastNodeIdToIncludeInCachedMap = -1;
    std::map<int, Transform> nodePoses;
    for (const auto& [nodeId, time] : nodeTimes_)
    {
        std::optional<Transform> pose = getPose(
            trajectories, time, activeTrajectoryPtr, extrapolationShift);
        if (pose.has_value())
        {
            auto nodeTrajectoryIt = trajectories.findCurrentTrajectory(time);
            if (activeTrajectoryPtr && nodeTrajectoryIt != trajectories.end() &&
                *nodeTrajectoryIt < *activeTrajectoryPtr)
            {
                lastNodeIdToIncludeInCachedMap = nodeId;
            }
            if (currentTrajectory_.size() &&
                currentTrajectory_.containsTime(time))
            {
                updatedCurrentTrajectory.addPose(time, *pose);
            }
            nodePoses[nodeId] = *pose;
        }
    }

    std::deque<Transform> temporaryNodePoses;
    for (const Time& time : temporaryNodeTimes_)
    {
        std::optional<Transform> pose = getPose(
            trajectories, time, activeTrajectoryPtr, extrapolationShift);
        UASSERT(pose.has_value());
        updatedCurrentTrajectory.addPose(time, *pose);
        temporaryNodePoses.push_back(*pose);
    }

    currentTrajectory_ = std::move(updatedCurrentTrajectory);

    ApproximatedPoses approximatedPoses;
    approximatedPoses.poses = std::move(nodePoses);
    approximatedPoses.temporaryPoses = std::move(temporaryNodePoses);
    approximatedPoses.lastNodeIdToIncludeInCachedMap = lastNodeIdToIncludeInCachedMap;
    return approximatedPoses;
}

std::optional<Transform> PosesApproximation::getPose(
    const Trajectories& trajectories, const Time& time,
    const Trajectory* activeTrajectoryPtr /* nullptr */,
    const std::optional<Transform>& extrapolationShift /* std::nullopt */)
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
    if (currentTrajectory_.size() &&
        currentTrajectory_.containsTime(time) &&
        activeTrajectoryPtr && extrapolationShift.has_value())
    {
        auto trajectoryIt = trajectories.findCurrentOrPreviousTrajectory(time);
        if (&(*trajectoryIt) == activeTrajectoryPtr)
        {
            std::optional<Transform> currentPose = currentTrajectory_.getPose(time);
            UASSERT(currentPose.has_value() || !currentTrajectory_.containsTime(time));
            if (currentPose.has_value())
            {
                Transform pose = *extrapolationShift * (*currentPose);
                return pose;
            }
        }
    }
    return std::nullopt;
}

void PosesApproximation::reset()
{
    currentTrajectory_.clear();
    nodeTimes_.clear();
    temporaryNodeTimes_.clear();
    lastNodeTime_ = Time();
    lastTemporaryNodeTime_ = Time();
}

void PosesApproximation::resetTemporary()
{
    temporaryNodeTimes_.clear();
    lastTemporaryNodeTime_ = Time();
}

}