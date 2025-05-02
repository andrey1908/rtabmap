#pragma once

#include <vector>
#include <list>
#include <map>

#include <yaml-cpp/yaml.h>

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Trajectory.h>
#include <rtabmap/core/LocalMap.h>
#include <rtabmap/core/Node.h>

namespace rtabmap {

class NodesSimilarityEstimation
{
public:
    NodesSimilarityEstimation() = default;

    float getSimilarity(
        const Transform& oldPose, const LocalMap2d& oldLocalMap,
        const Transform& newPose, const LocalMap2d::ColoredGrid& newGrid);
};

class PosesTrimmer
{
public:
    struct Parameters
    {
        float maxTimeErrorForClosestLocalMapSearch = 0.1;
        int skipLastN = 20;
        float maxDistance = 3.0f;
        float minSimilarity = 0.9f;

        static Parameters createParameters(const YAML::Node& node)
        {
            UASSERT(node.IsMap());
            Parameters parameters;
            if (node["MaxTimeErrorForClosestLocalMapSearch"])
            {
                parameters.maxTimeErrorForClosestLocalMapSearch =
                    node["MaxTimeErrorForClosestLocalMapSearch"].as<float>();
            }
            if (node["SkipLastN"])
            {
                parameters.skipLastN = node["SkipLastN"].as<int>();
            }
            if (node["MaxDistance"])
            {
                parameters.maxDistance = node["MaxDistance"].as<float>();
            }
            if (node["MinSimilarity"])
            {
                parameters.minSimilarity = node["MinSimilarity"].as<float>();
            }
            return parameters;
        }
    };

public:
    PosesTrimmer(const Parameters& parameters);
    void parseParameters(const Parameters& parameters);

    void addLocalMap(const std::shared_ptr<const LocalMap2d>& localMap);
    std::set<Time> getPosesToTrim(const Trajectories& trajectories);
    static Trajectories removePosesFromTrajectories(const Trajectories& trajectories,
        const std::set<Time>& posesToTrim);

private:
    const LocalMap2d* findNextClosestLocalMap(
        std::vector<std::shared_ptr<const LocalMap2d>>::const_iterator& it,
        const Time& time, double maxError) const;
    std::map<Time, const LocalMap2d*> findClosestLocalMaps(
        const Trajectories& trajectories, double maxError) const;

private:
    float maxTimeErrorForClosestLocalMapSearch_;
    int skipLastN_;
    float maxDistance_;
    float maxDistanceSqr_;
    float minSimilarity_;

    NodesSimilarityEstimation similarityEstimator_;

    std::vector<std::shared_ptr<const LocalMap2d>> localMaps_;
    Time prevLastTime_;
};

}
