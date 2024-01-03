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
    struct Parameters
    {
        float maxInterpolationTimeError = 0.06f;

        static Parameters createParameters(const YAML::Node& node)
        {
            UASSERT(node.IsMap());
            Parameters parameters;
            if (node["MaxInterpolationTimeError"])
            {
                parameters.maxInterpolationTimeError =
                    node["MaxInterpolationTimeError"].as<float>();
            }
            return parameters;
        }
    };

public:
    PosesApproximation(const Parameters& parameters);
    void parseParameters(const Parameters& parameters);

    void addTime(int nodeId, const Time& time);
    void removeTimes(const std::vector<int>& nodeIdsToRemove);

    // poses, lastNodeIdToIncludeInCachedMap
    std::pair<std::map<int, Transform>, int> approximatePoses(
        const Trajectories& trajectories) const;

    void reset();

private:
    std::optional<Transform> interpolate(
        const Trajectories& trajectories, const Time& time) const;

private:
    float maxInterpolationTimeError_;

    std::map<int, Time> times_;
};

}