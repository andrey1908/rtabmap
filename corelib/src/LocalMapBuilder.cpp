#include <rtabmap/core/LocalMapBuilder.h>
#include <rtabmap/utilite/ULogger.h>

#include <limits>

#include "time_measurer/time_measurer.h"

namespace rtabmap {

LocalMapBuilder::LocalMapBuilder(const ParametersMap& parameters) :
	cellSize_(Parameters::defaultGridCellSize()),
	maxRange_(Parameters::defaultLocalMapMaxRange()),
	minObstacleHeight_(Parameters::defaultLocalMapMinObstacleHeight()),
	maxObstacleHeight_(Parameters::defaultLocalMapMaxObstacleHeight()),
	minSemanticRange_(Parameters::defaultLocalMapMinSemanticRange()),
	maxSemanticRange_(Parameters::defaultLocalMapMaxSemanticRange()),
	useRayTracing_(Parameters::defaultLocalMapUseRayTracing())
{
	parseParameters(parameters);
}

void LocalMapBuilder::parseParameters(const ParametersMap& parameters)
{
	Parameters::parse(parameters, Parameters::kGridCellSize(), cellSize_);
	Parameters::parse(parameters, Parameters::kLocalMapMaxRange(), maxRange_);
	Parameters::parse(parameters, Parameters::kLocalMapMinObstacleHeight(), minObstacleHeight_);
	Parameters::parse(parameters, Parameters::kLocalMapMaxObstacleHeight(), maxObstacleHeight_);
	Parameters::parse(parameters, Parameters::kLocalMapMinSemanticRange(), minSemanticRange_);
	Parameters::parse(parameters, Parameters::kLocalMapMaxSemanticRange(), maxSemanticRange_);
	Parameters::parse(parameters, Parameters::kLocalMapUseRayTracing(), useRayTracing_);
	UASSERT(minObstacleHeight_ < maxObstacleHeight_);
	UASSERT(maxSemanticRange_ == 0.0f || minSemanticRange_ < maxSemanticRange_);

	maxRangeSqr_ = maxRange_ * maxRange_;
	minSemanticRangeSqr_ = minSemanticRange_ * minSemanticRange_;
	maxSemanticRangeSqr_ = maxSemanticRange_ * maxSemanticRange_;
	if (semanticDilation_ == nullptr)
	{
		semanticDilation_ = std::make_unique<SemanticDilation>(parameters);
	}
	else
	{
		semanticDilation_->parseParameters(parameters);
	}
	if (useRayTracing_)
	{
		if (rayTracing_ == nullptr)
		{
			rayTracing_ = std::make_unique<RayTracing>(parameters);
		}
		else
		{
			rayTracing_->parseParameters(parameters);
		}
	}
	else
	{
		rayTracing_.reset();
	}
}

LocalMapBuilder::LocalMap LocalMapBuilder::createLocalMap(const Signature& signature) const
{
	MEASURE_BLOCK_TIME(createLocalMap);
	UASSERT(!signature.sensorData().laserScan().isEmpty() &&
			!signature.sensorData().laserScan().is2d());
	const LaserScan& laserScan = signature.sensorData().laserScan();
	const Transform& transform = laserScan.localTransform();
	Eigen::Matrix3Xf points, obstacles;
	points = convertLaserScan(laserScan);
	points = filterMaxRange(points);
	points = transformPoints(points, transform);
	obstacles = getObstaclePoints(points);

	std::vector<Color> colors;
	if (signature.sensorData().images().size())
	{
		if (semanticDilation_->dilationSize() > 0)
		{
			std::vector<cv::Mat> dilatedImages;
			for (const cv::Mat& image : signature.sensorData().images())
			{
				cv::Mat dilatedImage = semanticDilation_->dilate(image);
				dilatedImages.push_back(dilatedImage);
				lastDilatedSemantic_ = dilatedImage;
			}
			colors = getPointsColors(obstacles, dilatedImages,
				signature.sensorData().cameraModels());
		}
		else
		{
			colors = getPointsColors(obstacles, signature.sensorData().images(),
				signature.sensorData().cameraModels());
		}
	}
	else
	{
		colors.resize(obstacles.cols(), Color::missingColor);
	}

	ColoredGrid coloredGrid;
	Eigen::Vector2f sensor;
	sensor.x() = transform.translation().x();
	sensor.y() = transform.translation().y();
	coloredGrid = coloredGridFromObstacles(obstacles, colors, sensor);
	if (useRayTracing_)
	{
		traceRays(coloredGrid, sensor);
	}
	
	LocalMap localMap = localMapFromColoredGrid(coloredGrid);
	return localMap;
}

Eigen::Matrix3Xf LocalMapBuilder::convertLaserScan(const LaserScan& laserScan) const
{
	MEASURE_BLOCK_TIME(____convertLaserScan);
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
	MEASURE_BLOCK_TIME(____filterMaxRange);
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
	MEASURE_BLOCK_TIME(____transformPoints);
	Eigen::Matrix3Xf transformed =
		(transform.toEigen3fRotation() * points).colwise() +
		transform.toEigen3fTranslation();
	return transformed;
}

Eigen::Matrix3Xf LocalMapBuilder::getObstaclePoints(const Eigen::Matrix3Xf& points) const
{
	MEASURE_BLOCK_TIME(____getObstaclePoints);
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

	Eigen::Matrix3Xf obstaclePoints(3, obstaclePointsIndices.size());
	int i = 0;
	for (int index : obstaclePointsIndices)
	{
		obstaclePoints(0, i) = points(0, index);
		obstaclePoints(1, i) = points(1, index);
		obstaclePoints(2, i) = points(2, index);
		i++;
	}
	return obstaclePoints;
}

std::vector<LocalMapBuilder::Color> LocalMapBuilder::getPointsColors(
	const Eigen::Matrix3Xf& points,
	const std::vector<cv::Mat>& images,
	const std::vector<CameraModel>& cameraModels) const
{
	MEASURE_BLOCK_TIME(____getPointsColors);
	UASSERT(images.size() == cameraModels.size());
	std::vector<Color> colors;
	colors.resize(points.cols(), Color::missingColor);
	for (int camI = 0; camI < images.size(); camI++)
	{
		const cv::Mat& image = images[camI];
		const CameraModel& cameraModel = cameraModels[camI];
		const Transform& transform = cameraModel.localTransform().inverse();
		const Eigen::Matrix3Xf& transformedPoints =
			(transform.toEigen3fRotation() * points).colwise() +
			transform.toEigen3fTranslation();
		for (int i = 0; i < transformedPoints.cols(); i++)
		{
			float x = transformedPoints(0, i);
			float y = transformedPoints(1, i);
			float z = transformedPoints(2, i);
			float rangeSqr = x * x + y * y + z * z;
			int u, v;
			cameraModel.reproject(x, y, z, u, v);
			if (cameraModel.inFrame(u, v) && z > 0 &&
				rangeSqr >= minSemanticRangeSqr_ &&
				(maxSemanticRangeSqr_ == 0.0f || rangeSqr <= maxSemanticRangeSqr_))
			{
				const std::uint8_t* pixelColor = image.ptr<std::uint8_t>(v, u);
				colors[i].b() = pixelColor[0];
				colors[i].g() = pixelColor[1];
				colors[i].r() = pixelColor[2];
				colors[i].missing() = false;
			}
		}
	}
	return colors;
}

LocalMapBuilder::ColoredGrid LocalMapBuilder::coloredGridFromObstacles(
	const Eigen::Matrix3Xf& points,
	const std::vector<Color>& colors,
	const Eigen::Vector2f& sensor) const
{
	MEASURE_BLOCK_TIME(____coloredGridFromObstacles);
	float minXf = sensor.x();
	float minYf = sensor.y();
	float maxXf = sensor.x();
	float maxYf = sensor.y();
	for (int i = 0; i < points.cols(); i++)
	{
		float x = points(0, i);
		float y = points(1, i);
		minXf = std::min(minXf, x);
		minYf = std::min(minYf, y);
		maxXf = std::max(maxXf, x);
		maxYf = std::max(maxYf, y);
	}
	if (useRayTracing_ && rayTracing_->traceRaysIntoUnknownSpace())
	{
		float range = rayTracing_->maxRayTracingRange();
		minXf = std::min(minXf, -range + sensor.x());
		minYf = std::min(minYf, -range + sensor.y());
		maxXf = std::max(maxXf,  range + sensor.x());
		maxYf = std::max(maxYf,  range + sensor.y());
	}

	ColoredGrid coloredGrid;
	coloredGrid.minX = std::floor(minXf / cellSize_);
	coloredGrid.minY = std::floor(minYf / cellSize_);
	int maxX = std::floor(maxXf / cellSize_);
	int maxY = std::floor(maxYf / cellSize_);
	int width = maxX - coloredGrid.minX + 1;
	int height = maxY - coloredGrid.minY + 1;
	coloredGrid.grid = cv::Mat(height, width, CV_8S, RayTracing::unknownCellValue);
	coloredGrid.colors = cv::Mat(height, width, CV_32S, Color::missingColor.data());
	for (int i = 0; i < points.cols(); i++)
	{
		float xf = points(0, i);
		float yf = points(1, i);
		int x = std::floor(xf / cellSize_) - coloredGrid.minX;
		int y = std::floor(yf / cellSize_) - coloredGrid.minY;
		coloredGrid.grid.at<std::int8_t>(y, x) = RayTracing::occupiedCellValue;
		const Color& pointColor = colors[i];
		if (pointColor != Color::missingColor)
		{
			Color& cellColor =
				reinterpret_cast<Color&>(coloredGrid.colors.at<int>(y, x));
			if (pointColor.brightness() > cellColor.brightness())
			{
				cellColor = pointColor;
			}
		}
	}
	return coloredGrid;
}

void LocalMapBuilder::traceRays(ColoredGrid& coloredGrid,
	const Eigen::Vector2f& sensor) const
{
	MEASURE_BLOCK_TIME(____traceRays);
	RayTracing::Cell origin;
	origin.y = std::floor(sensor.y() / cellSize_) - coloredGrid.minY;
	origin.x = std::floor(sensor.x() / cellSize_) - coloredGrid.minX;
	rayTracing_->traceRays(coloredGrid.grid, origin);
}

LocalMapBuilder::LocalMap LocalMapBuilder::localMapFromColoredGrid(
	const ColoredGrid& coloredGrid) const
{
	MEASURE_BLOCK_TIME(____localMapFromColoredGrid);
	std::vector<std::pair<int, int>> emptyCells;
	std::vector<std::pair<int, int>> occupiedCells;
	for (int y = 0; y < coloredGrid.grid.rows; y++)
	{
		for (int x = 0; x < coloredGrid.grid.cols; x++)
		{
			std::int8_t value = coloredGrid.grid.at<std::int8_t>(y, x);
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

	LocalMap localMap;
	localMap.numEmpty = emptyCells.size();
	localMap.numObstacles = occupiedCells.size();
	localMap.points.resize(3, localMap.numEmpty + localMap.numObstacles);
	localMap.colors.reserve(localMap.numEmpty + localMap.numObstacles);
	int i = 0;
	for (const std::pair<int, int> emptyCell : emptyCells)
	{
		int y = emptyCell.first;
		int x = emptyCell.second;
		localMap.points(0, i) = (x + coloredGrid.minX + 0.5f) * cellSize_;
		localMap.points(1, i) = (y + coloredGrid.minY + 0.5f) * cellSize_;
		localMap.points(2, i) = 0.0f;
		localMap.colors.push_back(
			reinterpret_cast<const Color&>(coloredGrid.colors.at<int>(y, x)));
		i++;
	}
	for (const std::pair<int, int> occupiedCell : occupiedCells)
	{
		int y = occupiedCell.first;
		int x = occupiedCell.second;
		localMap.points(0, i) = (x + coloredGrid.minX + 0.5f) * cellSize_;
		localMap.points(1, i) = (y + coloredGrid.minY + 0.5f) * cellSize_;
		localMap.points(2, i) = 0.0f;
		localMap.colors.push_back(
			reinterpret_cast<const Color&>(coloredGrid.colors.at<int>(y, x)));
		i++;
	}
	return localMap;
}

}
