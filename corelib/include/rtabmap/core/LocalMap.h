#pragma once

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Color.h>

#include <vector>
#include <memory>
#include <optional>
#include <Eigen/Core>

#include <rtabmap/proto/Color.pb.h>
#include <rtabmap/proto/Transform.pb.h>
#include <rtabmap/proto/LocalMap.pb.h>

namespace rtabmap {

struct LocalMap
{
	int numObstacles;
	int numEmpty;
	Eigen::Matrix3Xf points;  // z = 0
	std::vector<Color> colors;

	float sensorBlindRange2dSqr;
	Transform toSensor;

	// used to correct poses passed to updatePoses() function
	Transform fromUpdatedPose;
};

proto::LocalMap toProto(const LocalMap& localMap);
std::shared_ptr<LocalMap> fromProto(const proto::LocalMap& proto);

}