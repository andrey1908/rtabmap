#pragma once

namespace rtabmap {

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
    struct CompareId
    {
        using is_transparent = void;
        bool operator()(const Trajectory& a, const Trajectory& b) const
        {
            return a < b;
        }
        bool operator()(const Trajectory& a, const Time& time) const
        {
            return a.maxTime() < time;
        }
        bool operator()(const Time& time, const Trajectory& a) const
        {
            return time < a.minTime();
        }
    };

    using TimedPosesSet = std::set<TimedPose, TimedPose::CompareId>;
    using const_iterator = TimedPosesSet::const_iterator;
    using Bounds = std::pair<
        TimedPosesSet::const_iterator, TimedPosesSet::const_iterator>;

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
    bool containsTime(const Time& time) const
    {
        UASSERT(trajectory_.size());
        return minTime() <= time && time <= maxTime();
    }
    bool hasTime(const Time& time) const
    {
        UASSERT(trajectory_.size());
        return trajectory_.count(time);
    }
    Bounds getBounds(const Time& time) const
    {
        if (!containsTime(time))
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
    bool operator<(const Trajectory& other) const
    {
        return maxTime() < other.minTime();
    }
    const_iterator begin() const
    {
        return trajectory_.cbegin();
    }
    const_iterator end() const
    {
        return trajectory_.cend();
    }

private:
    TimedPosesSet trajectory_;
};


class Trajectories
{
public:
    using TrajectoriesSet = std::set<Trajectory, Trajectory::CompareId>;
    using const_iterator = TrajectoriesSet::const_iterator;

public:
    template<typename T>
    void addTrajectory(T&& trajectory)
    {
        bool emplaced = trajectories_.emplace(std::forward<T>(trajectory)).second;
        UASSERT(emplaced);
    }
    const_iterator findContainingTrajectory(const Time& time) const
    {
        return trajectories_.find(time);
    }
    const_iterator findContinuedTrajectory(const Time& time) const
    {
        auto it = trajectories_.lower_bound(time);
        if (it == trajectories_.begin())
        {
            trajectories_.end();
        }
        return std::prev(it);
    }
    size_t size() const
    {
        return trajectories_.size();
    }
    const_iterator begin() const
    {
        return trajectories_.cbegin();
    }
    const_iterator end() const
    {
        return trajectories_.cend();
    }

private:
    TrajectoriesSet trajectories_;
};

}
