#include <rtabmap/core/LocalMap.h>
#include <rtabmap/utilite/ULogger.h>

namespace rtabmap {

proto::LocalMap toProto(const LocalMap& localMap)
{
	proto::LocalMap proto;
	proto.set_num_obstacles(localMap.numObstacles);
	proto.set_num_empty(localMap.numEmpty);
	for (int i = 0; i < localMap.points.size(); i++)
	{
		proto.add_points(localMap.points.data()[i]);
	}
	for (const Color& color : localMap.colors)
	{
		*proto.add_colors() = toProto(color);
	}
	proto.set_sensor_blind_range_2d_sqr(localMap.sensorBlindRange2dSqr);
	UASSERT(!localMap.toSensor.isNull());
	*proto.mutable_to_sensor() = toProto(localMap.toSensor);
	UASSERT(!localMap.fromUpdatedPose.isNull());
	*proto.mutable_from_updated_pose() = toProto(localMap.fromUpdatedPose);
	return proto;
}

std::shared_ptr<LocalMap> fromProto(const proto::LocalMap& proto)
{
	auto localMap = std::make_shared<LocalMap>();
	localMap->numObstacles = proto.num_obstacles();
	localMap->numEmpty = proto.num_empty();
	int numPoints = proto.num_obstacles() + proto.num_empty();
	localMap->points.resize(3, numPoints);
	for (int i = 0; i < proto.points_size(); i++)
	{
		localMap->points.data()[i] = proto.points(i);
	}
	localMap->colors.reserve(numPoints);
	for (int i = 0; i < proto.colors_size(); i++)
	{
		localMap->colors.emplace_back(fromProto(proto.colors(i)));
	}
	localMap->sensorBlindRange2dSqr = proto.sensor_blind_range_2d_sqr();
	UASSERT(proto.has_to_sensor());
	localMap->toSensor = fromProto(proto.to_sensor());
	UASSERT(proto.has_from_updated_pose());
	localMap->fromUpdatedPose = fromProto(proto.from_updated_pose());
	return localMap;
}

}
