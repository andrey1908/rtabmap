#include <rtabmap/core/LocalMapBuilder.h>
#include <rtabmap/utilite/ULogger.h>

#include <limits>

#include "time_measurer/time_measurer.h"

namespace rtabmap {

LocalMapBuilder::LocalMapBuilder(const ParametersMap& parameters) :
	cellSize_(Parameters::defaultGridCellSize()),
	maxRange_(Parameters::defaultGridMaxRange()),
	minObstacleHeight_(Parameters::defaultGridMinObstacleHeight()),
	maxObstacleHeight_(Parameters::defaultGridMaxObstacleHeight()),
	useRayTracing_(Parameters::defaultGridRayTracing()),
	rayTracing_(parameters)
{
	parseParameters(parameters);
	maxRangeSqr_ = maxRange_ * maxRange_;
}

void LocalMapBuilder::parseParameters(const ParametersMap& parameters)
{
	Parameters::parse(parameters, Parameters::kGridCellSize(), cellSize_);
	Parameters::parse(parameters, Parameters::kGridMaxRange(), maxRange_);
	Parameters::parse(parameters, Parameters::kGridMinObstacleHeight(), minObstacleHeight_);
	Parameters::parse(parameters, Parameters::kGridMaxObstacleHeight(), maxObstacleHeight_);
	Parameters::parse(parameters, Parameters::kGridRayTracing(), useRayTracing_);
	UASSERT(minObstacleHeight_ < maxObstacleHeight_);
}

LocalMapBuilder::LocalMap LocalMapBuilder::createLocalMap(const Signature& signature) const
{
	MEASURE_BLOCK_TIME(createLocalMap);
	UASSERT(!signature.sensorData().laserScan().isEmpty() &&
			!signature.sensorData().laserScan().is2d());
	const LaserScan& laserScan = signature.sensorData().laserScan();
	const Transform& transform = laserScan.localTransform();
	Eigen::Matrix3Xf points;
	Eigen::Matrix2Xf points2d;
	points = convertLaserScan(laserScan);
	points = filterMaxRange(points);
	points = transformPoints(points, transform);
	points2d = getObstaclePoints(points);

	int minY, minX;
	cv::Mat grid;
	Eigen::Vector2f sensor;
	sensor(0) = transform.translation().x();
	sensor(1) = transform.translation().y();
	grid = gridFromObstacles(points2d, sensor, minY, minX);
	if (useRayTracing_)
	{
		traceRays(grid, sensor, minY, minX);
	}
	
	LocalMap localMap = localMapFromGrid(grid, minY, minX);
	return localMap;
}

Eigen::Matrix3Xf LocalMapBuilder::convertLaserScan(const LaserScan& laserScan) const
{
	Eigen::Matrix3Xf points(3, laserScan.size());
	for (int i = 0; i < laserScan.size(); i++)
	{
		float x = laserScan.data().ptr<float>(0, i)[0];
		float y = laserScan.data().ptr<float>(0, i)[1];
		float z = laserScan.data().ptr<float>(0, i)[2];
		points(0, i) = x;
		points(1, i) = y;
		points(2, i) = z;
	}
	return points;
}

Eigen::Matrix3Xf LocalMapBuilder::filterMaxRange(const Eigen::Matrix3Xf& points) const
{
	std::vector<int> indices;
	indices.reserve(points.cols());
	for (int i = 0; i < points.cols(); i++)
	{
		float x = points(0, i);
		float y = points(1, i);
		float z = points(2, i);
		if (x * x + y * y + z * z <= maxRangeSqr_)
		{
			indices.push_back(i);
		}
	}

	Eigen::Matrix3Xf filtered(3, indices.size());
	int i = 0;
	for (int index : indices)
	{
		filtered(0, i) = points(0, index);
		filtered(1, i) = points(1, index);
		filtered(2, i) = points(2, index);
		i++;
	}
	return filtered;
}

Eigen::Matrix3Xf LocalMapBuilder::transformPoints(const Eigen::Matrix3Xf& points,
	const Transform& transform) const
{
	Eigen::Matrix3Xf transformed =
		(transform.toEigen3fRotation() * points).colwise() +
		transform.toEigen3fTranslation();
	return transformed;
}

Eigen::Matrix2Xf LocalMapBuilder::getObstaclePoints(const Eigen::Matrix3Xf& points) const
{
	std::vector<int> obstaclePointsIndices;
	obstaclePointsIndices.reserve(points.cols());
	for (int i = 0; i < points.cols(); i++)
	{
		float z = points(2, i);
		if (minObstacleHeight_ <= z && z <= maxObstacleHeight_)
		{
			obstaclePointsIndices.push_back(i);
		}
	}

	Eigen::Matrix2Xf obstaclePoints(2, obstaclePointsIndices.size());
	int i = 0;
	for (int index : obstaclePointsIndices)
	{
		obstaclePoints(0, i) = points(0, index);
		obstaclePoints(1, i) = points(1, index);
		i++;
	}
	return obstaclePoints;
}

cv::Mat LocalMapBuilder::gridFromObstacles(const Eigen::Matrix2Xf& points,
	const Eigen::Vector2f& sensor, int& minY, int& minX) const
{
	float minXf = sensor(0);
	float minYf = sensor(1);
	float maxXf = sensor(0);
	float maxYf = sensor(1);
	for (int i = 0; i < points.cols(); i++)
	{
		float x = points(0, i);
		float y = points(1, i);
		minXf = std::min(minXf, x);
		minYf = std::min(minYf, y);
		maxXf = std::max(maxXf, x);
		maxYf = std::max(maxYf, y);
	}

	minX = std::floor(minXf / cellSize_);
	minY = std::floor(minYf / cellSize_);
	int maxX = std::floor(maxXf / cellSize_);
	int maxY = std::floor(maxYf / cellSize_);
	int width = maxX - minX + 1;
	int height = maxY - minY + 1;
	cv::Mat grid(height, width, CV_8SC1, (std::int8_t)(-1));
	for (int i = 0; i < points.cols(); i++)
	{
		float xf = points(0, i);
		float yf = points(1, i);
		int x = std::floor(xf / cellSize_) - minX;
		int y = std::floor(yf / cellSize_) - minY;
		grid.at<std::int8_t>(y, x) = RayTracing::occupiedCellValue;
	}
	return grid;
}

void LocalMapBuilder::traceRays(cv::Mat& grid,
	const Eigen::Vector2f& sensor, int minY, int minX) const
{
	RayTracing::Cell origin;
	origin.y = std::floor(sensor(1) / cellSize_) - minY;
	origin.x = std::floor(sensor(0) / cellSize_) - minX;
	rayTracing_.traceRays(grid, origin);
}

LocalMapBuilder::LocalMap LocalMapBuilder::localMapFromGrid(const cv::Mat& grid,
	int minY, int minX) const
{
	LocalMap localMap;
	std::vector<std::pair<int, int>> emptyCells;
	std::vector<std::pair<int, int>> occupiedCells;
	for (int y = 0; y < grid.rows; y++)
	{
		for (int x = 0; x < grid.cols; x++)
		{
			std::int8_t value = grid.at<std::int8_t>(y, x);
			if (value == RayTracing::occupiedCellValue)
			{
				occupiedCells.emplace_back(y, x);
			}
			else if (value == RayTracing::emptyCellValue)
			{
				emptyCells.emplace_back(y, x);
			}
		}
	}

	localMap.numEmpty = emptyCells.size();
	localMap.numObstacles = occupiedCells.size();
	localMap.points.resize(3, localMap.numEmpty + localMap.numObstacles);
	int i = 0;
	for (const std::pair<int, int> emptyCell : emptyCells)
	{
		int y = emptyCell.first;
		int x = emptyCell.second;
		localMap.points(0, i) = (x + 0.5f) * cellSize_;
		localMap.points(1, i) = (y + 0.5f) * cellSize_;
		localMap.points(2, i) = 0.0f;
		i++;
	}
	localMap.colors.resize(localMap.points.cols(), -1);
	return localMap;
}

}
