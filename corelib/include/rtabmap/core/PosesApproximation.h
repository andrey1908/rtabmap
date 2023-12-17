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
        float guaranteedInterpolationTimeWindow = 1.0f;

        static Parameters createParameters(const YAML::Node& node)
        {
            UASSERT(node.IsMap());
            Parameters parameters;
            if (node["MaxInterpolationTimeError"])
            {
                parameters.maxInterpolationTimeError =
                    node["MaxInterpolationTimeError"].as<float>();
            }
            if (node["GuaranteedInterpolationTimeWindow"])
            {
                parameters.guaranteedInterpolationTimeWindow =
                    node["GuaranteedInterpolationTimeWindow"].as<float>();
            }
            return parameters;
        }
    };

    struct ApproximatedPoses
    {
        std::map<int, Transform> poses;
        std::deque<Transform> temporaryPoses;
        int lastNodeIdToIncludeInCachedMap;
    };

public:
    PosesApproximation(const Parameters& parameters);
    void parseParameters(const Parameters& parameters);

    void addNode(int nodeId, const Time& time);
    void addNode(int nodeId, const Time& time, const Transform& pose);
    void addTemporaryNode(const Time& time, const Transform& pose);

    void removeNodes(const std::vector<int>& nodeIdsToRemove);
    void removeFirstTemporaryNode();

    void transformCurrentTrajectory(const Transform& transform);

    ApproximatedPoses approximatePoses(const Trajectories& trajectories);

    void reset();
    void resetTemporary();

private:
    std::optional<Transform> getPose(
        const Trajectories& trajectories, const Time& time,
        const Trajectory* activeTrajectoryPtr = nullptr,
        const std::optional<Transform>& extrapolationShift = std::nullopt);

private:
    float maxInterpolationTimeError_;
    float guaranteedInterpolationTimeWindow_;

    Trajectory currentTrajectory_;

    std::map<int, Time> nodeTimes_;
    std::deque<Time> temporaryNodeTimes_;

    Time lastNodeTime_;
    Time lastTemporaryNodeTime_;
};

}