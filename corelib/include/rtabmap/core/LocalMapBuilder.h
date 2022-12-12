#pragma once

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/RayTracing.h>

#include <pcl/point_cloud.h>

#include <vector>
#include <utility>
#include <Eigen/Core>

namespace rtabmap {

class LocalMapBuilder
{
public:
	struct LocalMap
	{
		int numEmpty, numObstacles;
		Eigen::Matrix3Xf points;  // z = 0
		std::vector<int> colors;
	};

public:
	LocalMapBuilder();
	LocalMapBuilder(const ParametersMap& parameters);
	void setDefaultParameters();
	void updateParameters(const ParametersMap& parameters);

    LocalMap createLocalMap(const Signature& signature) const;

private:
	void initializeDefaultParameters();
	void precompute();

	Eigen::Matrix3Xf convertLaserScan(const LaserScan& laserScan) const;
	Eigen::Matrix3Xf filterMaxRange(const Eigen::Matrix3Xf& points) const;
	Eigen::Matrix3Xf transformPoints(const Eigen::Matrix3Xf& points,
		const Transform& transform) const;
	Eigen::Matrix2Xf getObstaclePoints(const Eigen::Matrix3Xf& points) const;
	cv::Mat gridFromObstacles(const Eigen::Matrix2Xf& points,
		const Eigen::Vector2f& sensor, int& minY, int& minX) const;
	void traceRays(cv::Mat& grid,
		const Eigen::Vector2f& sensor, int minY, int minX) const;
	LocalMap localMapFromGrid(const cv::Mat& grid, int minY, int minX) const;

private:
	float cellSize_;
	float maxRange_;
	float maxRangeSqr_;
	float minObstacleHeight_;
	float maxObstacleHeight_;
	bool useRayTracing_;

	RayTracing rayTracing_;
};

}
