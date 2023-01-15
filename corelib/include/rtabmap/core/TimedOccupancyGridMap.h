#pragma once

#include <rtabmap/core/Time.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/OccupancyGridMap.h>
#include <rtabmap/utilite/ULogger.h>

#include <vector>
#include <set>
#include <map>
#include <optional>
#include <utility>
#include <memory>

namespace rtabmap {

class TimedOccupancyGridMap
{
public:
    struct TimedPose
    {
        struct CompareId
        {
            using is_transparent = void;
            bool operator()(const TimedPose& a, const TimedPose& b) const
            {
                return a.time < b.time;
            }
            bool operator()(const TimedPose& a, const Time& time) const
            {
                return a.time < time;
            }
            bool operator()(const Time& time, const TimedPose& a) const
            {
                return time < a.time;
            }
        };

        template<typename T, typename U>
        TimedPose(T&& otherTime, U&& otherPose) :
            time(std::forward<T>(otherTime)),
            pose(std::forward<U>(otherPose)) {}
        Time time;
        Transform pose;
    };

    class Trajectory
    {
    public:
        void addPose(const Time& time, const Transform& pose)
        {
            bool emplaced = trajectory_.emplace(time, pose).second;
            UASSERT(emplaced);
        }
        Time minTime() const
        {
            UASSERT(trajectory_.size());
            return trajectory_.begin()->time;
        }
        Time maxTime() const
        {
            UASSERT(trajectory_.size());
            return trajectory_.rbegin()->time;
        }
        bool includesTime(const Time& time) const
        {
            UASSERT(trajectory_.size());
            return minTime() <= time && time <= maxTime();
        }
        std::pair<
            std::set<TimedPose>::const_iterator,
            std::set<TimedPose>::const_iterator>
                getBounds(const Time& time) const
        {
            if (!includesTime(time))
            {
                return std::make_pair(trajectory_.cend(), trajectory_.cend());
            }
            auto it = trajectory_.upper_bound(time);
            return std::make_pair(std::prev(it), it);
        }
        size_t size() const
        {
            return trajectory_.size();
        }
        std::set<TimedPose>::const_iterator begin() const
        {
            return trajectory_.cbegin();
        }
        std::set<TimedPose>::const_iterator end() const
        {
            return trajectory_.cend();
        }

    private:
        std::set<TimedPose, TimedPose::CompareId> trajectory_;
    };

    // assumes that trajectoies are not overlapping
    class Trajectories
    {
    public:
        int appendTrajectory()
        {
            trajectories_.emplace_back();
            return trajectories_.size() - 1;
        }
        void addPose(int trajectory_index, const Time& time, const Transform& pose)
        {
            UASSERT(trajectory_index < (int)trajectories_.size());
            trajectories_[trajectory_index].addPose(time, pose);
        }
        const Trajectory* getLatestTrajectory() const
        {
            if (trajectories_.empty())
            {
                return nullptr;
            }
            const Trajectory* latestTrajectory = &trajectories_.front();
            Time maxTime = trajectories_.front().maxTime();
            for (const auto& trajectory : trajectories_)
            {
                if (trajectory.maxTime() > maxTime)
                {
                    latestTrajectory = &trajectory;
                    maxTime = trajectory.maxTime();
                }
            }
            return latestTrajectory;
        }
        const Trajectory* findContinuedTrajectory(const Time& time) const
        {
            // assumes that there are no trajectories containing time
            double minDiff = std::numeric_limits<double>::max();
            const Trajectory* continuedTrajectory = nullptr;
            for (const Trajectory& trajectory : trajectories_)
            {
                if (trajectory.maxTime() > time)
                {
                    continue;
                }
                double diff = time.toSec() - trajectory.maxTime().toSec();
                if (diff < minDiff)
                {
                    minDiff = diff;
                    continuedTrajectory = &trajectory;
                }
            }
            return continuedTrajectory;
        }
        size_t size() const
        {
            return trajectories_.size();
        }
        std::vector<Trajectory>::const_iterator begin() const {
            return trajectories_.cbegin();
        }
        std::vector<Trajectory>::const_iterator end() const {
            return trajectories_.cend();
        }

    private:
        std::vector<Trajectory> trajectories_;
    };

public:
    TimedOccupancyGridMap(const ParametersMap& parameters = ParametersMap());
    void parseParameters(const ParametersMap& parameters);

    std::shared_ptr<LocalMap> createLocalMap(const Signature& signature,
        const Transform& fromUpdatedPose = Transform::getIdentity()) const
            { return occupancyGridMap_->createLocalMap(signature, fromUpdatedPose); }

    void addLocalMap(int nodeId, const Time& time,
        std::shared_ptr<const LocalMap> localMap);
    void addLocalMap(int nodeId, const Time& time,
        const Transform& pose, std::shared_ptr<const LocalMap> localMap);
    void addTemporaryLocalMap(const Time& time,
        const Transform& pose, std::shared_ptr<const LocalMap> localMap);

    void cacheCurrentMap() { occupancyGridMap_->cacheCurrentMap(); }

    void updatePoses(const Trajectories& trajectories,
        bool autoCaching = false);

    OccupancyGrid getOccupancyGrid() const
        { return occupancyGridMap_->getOccupancyGrid(); }
    OccupancyGrid getProbOccupancyGrid() const
        { return occupancyGridMap_->getProbOccupancyGrid(); }
    ColorGrid getColorGrid() const
        { return occupancyGridMap_->getColorGrid(); }

    float cellSize() const { return occupancyGridMap_->cellSize(); }
    std::pair<float, float> getGridOrigin() const
        { return occupancyGridMap_->getGridOrigin(); }
    int maxTemporaryLocalMaps() const
        { return occupancyGridMap_->maxTemporaryLocalMaps(); }
    const std::map<int, Node>& nodes() const { return occupancyGridMap_->nodes(); }
    const std::map<int, const std::shared_ptr<const LocalMap>>&
        localMapsWithoutObstacleDilation() const
            { return occupancyGridMap_->localMapsWithoutObstacleDilation(); }
    const cv::Mat& lastDilatedSemantic() const
        { return occupancyGridMap_->lastDilatedSemantic(); }

    std::optional<Transform> getNodePose(int nodeId) const
        { return occupancyGridMap_->getNodePose(nodeId); }
    Transform getTemporaryNodePose(int index) const
        { return occupancyGridMap_->getTemporaryNodePose(index); }

    void reset();

    void save(const std::string& file);
    int load(const std::string& file);

private:
    std::pair<std::optional<Transform>, const Trajectory*> getPose(
        const Trajectories& trajectories, const Time& time,
        std::optional<Transform> prevPose = std::nullopt);
    std::optional<Transform> getPose(
        const Trajectory& trajectory, const Time& time);

private:
    double maxInterpolationTimeError_;
    double guaranteedInterpolationTimeWindow_;

    std::map<int, Time> times_;
    std::list<Time> temporaryTimes_;
    std::unique_ptr<OccupancyGridMap> occupancyGridMap_;

    Trajectories prevTrajectories_;
};

}
