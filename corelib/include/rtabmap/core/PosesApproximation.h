#pragma once

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/Time.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Trajectory.h>

#include <yaml-cpp/yaml.h>

#include <deque>
#include <map>
#include <optional>
#include <utility>

namespace rtabmap {

class PosesApproximation
{
public:
    PosesApproximation() = default;

    void addLocalPose(const Time& time, const Transform& localPose);
    void addTime(int nodeId, const Time& time);
    void removeTimes(const std::vector<int>& nodeIdsToRemove);

    // poses, lastNodeIdToIncludeInCachedMap
    std::pair<std::map<int, Transform>, int> approximatePoses(
        const Trajectories& trajectories) const;

    const Trajectory& localPoses() const { return localPoses_; }

    void reset();

private:
    std::optional<Transform> approximate(
        const Trajectories& trajectories, const Time& time, int nodeId /* for logs */) const;

private:
    float maxInterpolationTimeError_;

    Trajectory localPoses_;
    std::map<int, Time> times_;
};

}