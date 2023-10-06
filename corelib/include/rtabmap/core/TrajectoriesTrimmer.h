#pragma once

#include <vector>
#include <list>
#include <map>

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
        const Transform& oldPose, const LocalMap& oldLocalMap,
        const Transform& newPose, const LocalMap::ColoredGrid& newGrid);    
};

class TrajectoriesTrimmer
{
public:
    TrajectoriesTrimmer(int skipLastN, float maxDistance, float minSimilarity);

    void addLocalMap(const std::shared_ptr<const LocalMap>& localMap);
    std::set<Time> trimTrajectories(const Trajectories& trajectories);
    static Trajectories getTrimmedTrajectories(const Trajectories& trajectories,
        const std::set<Time>& posesToTrim);

private:
    const LocalMap* findNextClosestLocalMap(
        std::vector<std::shared_ptr<const LocalMap>>::const_iterator& it,
        const Time& time, double maxDiff) const;
    std::map<Time, const LocalMap*> findClosestLocalMaps(
        const Trajectories& trajectories, double maxDiff) const;

private:
    int skipLastN_;
    float maxDistance_;
    float maxDistanceSqr_;
    float minSimilarity_;

    NodesSimilarityEstimation similarityEstimator_;

    std::vector<std::shared_ptr<const LocalMap>> localMaps_;
    Time prevLastTime_;
};

}
