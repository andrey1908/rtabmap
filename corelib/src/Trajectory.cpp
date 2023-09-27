#include <rtabmap/core/Trajectory.h>

namespace rtabmap {

Trajectory::Bounds Trajectory::getBounds(const Time& time) const
{
    if (!containsTime(time))
    {
        return std::make_pair(trajectory_.cend(), trajectory_.cend());
    }
    auto it = trajectory_.upper_bound(time);
    return std::make_pair(std::prev(it), it);
}

std::optional<Transform> Trajectory::interpolate(const Time& time,
    double maxInterpolationTimeError /* std::numeric_limits<double>::max() */) const
{
    const Trajectory::Bounds& bounds = getBounds(time);
    if (bounds.first == end())
    {
        return std::nullopt;
    }
    if (bounds.first->time == time)
    {
        return bounds.first->pose;
    }

    UASSERT(bounds.second != end());
    if (std::min(
            std::abs(time.toSec() - bounds.first->time.toSec()),
            std::abs(time.toSec() - bounds.second->time.toSec())) >
                maxInterpolationTimeError)
    {
        return std::nullopt;
    }

    float t = (time.toSec() - bounds.first->time.toSec()) /
        (bounds.second->time.toSec() - bounds.first->time.toSec());
    UASSERT(t > 0.0 && t < 1.0);
    const Transform& interpolated =
        bounds.first->pose.interpolate(t, bounds.second->pose);
    return interpolated;
}

void Trajectory::trim(const Time& time)
{
    auto it = trajectory_.upper_bound(time);
    if (it == trajectory_.begin())
    {
        return;
    }
    trajectory_.erase(trajectory_.begin(), std::prev(it));
}

proto::TimedPose toProto(const TimedPose& timedPose)
{
    proto::TimedPose proto;
    *proto.mutable_time() = toProto(timedPose.time);
    *proto.mutable_pose() = toProto(timedPose.pose);
    return proto;
}

TimedPose fromProto(const proto::TimedPose& proto)
{
    Time time = fromProto(proto.time());
    Transform pose = fromProto(proto.pose());
    TimedPose timedPose(std::move(time), std::move(pose));
    return timedPose;
}

proto::Trajectory toProto(const Trajectory& trajectory)
{
    proto::Trajectory proto;
    proto.mutable_timed_poses()->Reserve(trajectory.size());
    for (const TimedPose& timedPose : trajectory)
    {
        *proto.add_timed_poses() = toProto(timedPose);
    }
    return proto;
}

Trajectory fromProto(const proto::Trajectory& proto)
{
    Trajectory trajectory;
    for (const proto::TimedPose& timedPoseProto : proto.timed_poses())
    {
        TimedPose timedPose = fromProto(timedPoseProto);
        trajectory.addPose(timedPose.time, timedPose.pose);
    }
    return trajectory;
}

proto::Trajectories toProto(const Trajectories& trajectories)
{
    proto::Trajectories proto;
    proto.mutable_trajectories()->Reserve(trajectories.size());
    for (const Trajectory& trajectory : trajectories)
    {
        *proto.add_trajectories() = toProto(trajectory);
    }
    return proto;
}

Trajectories fromProto(const proto::Trajectories& proto)
{
    Trajectories trajectories;
    for (const proto::Trajectory& trajectoryProto : proto.trajectories())
    {
        trajectories.addTrajectory(fromProto(trajectoryProto));
    }
    return trajectories;
}

}
