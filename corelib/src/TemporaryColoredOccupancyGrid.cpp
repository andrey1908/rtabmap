#include <rtabmap/core/TemporaryColoredOccupancyGrid.h>
#include <rtabmap/utilite/ULogger.h>

#include "time_measurer/time_measurer.h"

namespace rtabmap {

TemporaryColoredOccupancyGrid::TemporaryColoredOccupancyGrid(const ParametersMap& parameters) :
	cellSize_(Parameters::defaultGridCellSize()),
	temporaryMissProb_(Parameters::defaultGridTemporaryMissProb()),
	temporaryHitProb_(Parameters::defaultGridTemporaryHitProb()),
	temporaryOccupancyProbThr_(Parameters::defaultGridTemporaryOccupancyProbThr()),
	maxTemporaryLocalMaps_(Parameters::defaultGridMaxTemporaryLocalMaps())
{
	parseParameters(parameters);
}

void TemporaryColoredOccupancyGrid::parseParameters(const ParametersMap & parameters)
{
	Parameters::parse(parameters, Parameters::kGridCellSize(), cellSize_);
	Parameters::parse(parameters, Parameters::kGridTemporaryMissProb(), temporaryMissProb_);
	Parameters::parse(parameters, Parameters::kGridTemporaryHitProb(), temporaryHitProb_);
	Parameters::parse(parameters, Parameters::kGridTemporaryOccupancyProbThr(), temporaryOccupancyProbThr_);
	Parameters::parse(parameters, Parameters::kGridMaxTemporaryLocalMaps(), maxTemporaryLocalMaps_);
	UASSERT(temporaryMissProb_ > 0.0f && temporaryMissProb_ <= 0.5f);
	UASSERT(temporaryHitProb_ >= 0.5f && temporaryHitProb_ < 1.0f);
	UASSERT(temporaryOccupancyProbThr_ > 0.0f && temporaryOccupancyProbThr_ < 1.0f);
	UASSERT(maxTemporaryLocalMaps_ >= 1);

	miss_ = logodds(temporaryMissProb_);
	hit_ = logodds(temporaryHitProb_);
	occupancyThr_ = logodds(temporaryOccupancyProbThr_);
}

void TemporaryColoredOccupancyGrid::addLocalMap(const Transform& pose, LocalMap localMap)
{
	TransformedLocalMap transformedLocalMap = transformLocalMap(localMap, pose);
	Node node(std::move(localMap), std::move(transformedLocalMap));
	nodes_.emplace_back(std::move(node));
	const Node& newNode = nodes_.back();
	MapLimits newMapLimits = MapLimits::unite(mapLimits_,
		newNode.transformedLocalMap->mapLimits);
	if (mapLimits_ != newMapLimits)
	{
		createOrResizeMap(newMapLimits);
	}
	deployLastLocalMap();
	if (nodes_.size() > maxTemporaryLocalMaps_)
	{
		removeFirstLocalMap();
	}
}

void TemporaryColoredOccupancyGrid::updatePoses(const std::list<Transform>& updatedPoses)
{
	MEASURE_BLOCK_TIME(TemporaryColoredOccupancyGrid__updatePoses);
	UASSERT(nodes_.size() == updatedPoses.size());
	clear();
	MapLimits newMapLimits = MapLimits();
	auto updatedPoseIt = updatedPoses.begin();
	for (Node& node : nodes_)
	{
		node.transformedLocalMap = transformLocalMap(node.localMap, *updatedPoseIt);
		newMapLimits = MapLimits::unite(newMapLimits, node.transformedLocalMap->mapLimits);
		++updatedPoseIt;
	}
	if (newMapLimits.valid())
	{
		createOrResizeMap(newMapLimits);
		for (const Node& node : nodes_)
		{
			deployLocalMap(node);
		}
	}
}

TemporaryColoredOccupancyGrid::TransformedLocalMap TemporaryColoredOccupancyGrid::transformLocalMap(
	const LocalMap& localMap, const Transform& transform)
{
	MEASURE_BLOCK_TIME(OccupancyGrid__transformLocalMap);
	TransformedLocalMap transformedLocalMap;
	transformedLocalMap.pose = transform;
	transformedLocalMap.points.resize(2, localMap.points.cols());
	Eigen::Matrix3Xf transformedPoints =
		(transform.toEigen3fRotation() * localMap.points).colwise() +
		transform.toEigen3fTranslation();
	transformedLocalMap.mapLimits = MapLimits();
	for (int i = 0; i < transformedPoints.cols(); i++)
	{
		int x = std::floor(transformedPoints(0, i) / cellSize_);
		int y = std::floor(transformedPoints(1, i) / cellSize_);
		transformedLocalMap.points.coeffRef(0, i) = x;
		transformedLocalMap.points.coeffRef(1, i) = y;
		transformedLocalMap.mapLimits.update(x, y);
	}
	return transformedLocalMap;
}

void TemporaryColoredOccupancyGrid::createOrResizeMap(const MapLimits& newMapLimits)
{
	UASSERT(newMapLimits.valid());
	if(!mapLimits_.valid())
	{
		MEASURE_BLOCK_TIME(TemporaryColoredOccupancyGrid__createOrResizeMap__createMap);
		mapLimits_ = newMapLimits;
		missCounter_ = Eigen::MatrixXi::Constant(newMapLimits.height(), newMapLimits.width(),
			0);
		hitCounter_ = Eigen::MatrixXi::Constant(newMapLimits.height(), newMapLimits.width(),
			0);
		colors_ = Eigen::MatrixXi::Constant(newMapLimits.height(), newMapLimits.width(),
			LocalMapBuilder::Color::missingColor.data());
	}
	else if(mapLimits_ != newMapLimits)
	{
		MEASURE_BLOCK_TIME(TemporaryColoredOccupancyGrid__createOrResizeMap__resizeMap);
		int dstShiftX = std::max(mapLimits_.minX - newMapLimits.minX, 0);
		int dstShiftY = std::max(mapLimits_.minY - newMapLimits.minY, 0);
		int srcShiftX = std::max(newMapLimits.minX - mapLimits_.minX, 0);
		int srcShiftY = std::max(newMapLimits.minY - mapLimits_.minY, 0);
		MapLimits intersection = MapLimits::intersect(mapLimits_, newMapLimits);
		int copyWidth = intersection.width();
		int copyHeight = intersection.height();
		UASSERT(copyWidth > 0 && copyHeight > 0);

		Eigen::MatrixXi newMissCounter = Eigen::MatrixXi::Constant(newMapLimits.height(), newMapLimits.width(),
			0);
		Eigen::MatrixXi newHitCounter = Eigen::MatrixXi::Constant(newMapLimits.height(), newMapLimits.width(),
			0);
		Eigen::MatrixXi newColors = Eigen::MatrixXi::Constant(newMapLimits.height(), newMapLimits.width(),
			LocalMapBuilder::Color::missingColor.data());

		newMissCounter.block(dstShiftY, dstShiftX, copyHeight, copyWidth) =
			missCounter_.block(srcShiftY, srcShiftX, copyHeight, copyWidth);
		newHitCounter.block(dstShiftY, dstShiftX, copyHeight, copyWidth) =
			hitCounter_.block(srcShiftY, srcShiftX, copyHeight, copyWidth);
		newColors.block(dstShiftY, dstShiftX, copyHeight, copyWidth) =
			colors_.block(srcShiftY, srcShiftX, copyHeight, copyWidth);

		mapLimits_ = newMapLimits;
		missCounter_ = std::move(newMissCounter);
		hitCounter_ = std::move(newHitCounter);
		colors_ = std::move(newColors);
	}
}

void TemporaryColoredOccupancyGrid::deployLastLocalMap()
{
	UASSERT(nodes_.size());
	deployLocalMap(nodes_.back());
}

void TemporaryColoredOccupancyGrid::deployLocalMap(const Node& node)
{
	MEASURE_BLOCK_TIME(TemporaryColoredOccupancyGrid__deployLocalMap);
	UASSERT(node.transformedLocalMap.has_value());
	const Eigen::Matrix2Xi& transformedPoints = node.transformedLocalMap->points;
	for (int i = 0; i < transformedPoints.cols(); i++)
	{
		int x = transformedPoints.coeff(0, i) - mapLimits_.minX;
		int y = transformedPoints.coeff(1, i) - mapLimits_.minY;
		UASSERT(x >= 0 && x < missCounter_.cols() && y >= 0 && y < missCounter_.rows());

		bool free = (i < node.localMap.numEmpty);
		if (free)
		{
			missCounter_.coeffRef(y, x) += 1;
		}
		else
		{
			hitCounter_.coeffRef(y, x) += 1;
		}

		LocalMapBuilder::Color localMapColor = node.localMap.colors[i];
		colors_.coeffRef(y, x) = localMapColor.data();
	}
}

void TemporaryColoredOccupancyGrid::removeFirstLocalMap()
{
	removeLocalMap(nodes_.begin());
}

void TemporaryColoredOccupancyGrid::removeLocalMap(std::list<Node>::iterator nodeIt)
{
	MEASURE_BLOCK_TIME(TemporaryColoredOccupancyGrid__removeLocalMap);
	UASSERT(nodeIt->transformedLocalMap.has_value());
	const Eigen::Matrix2Xi& transformedPoints = nodeIt->transformedLocalMap->points;
	for (int i = 0; i < transformedPoints.cols(); i++)
	{
		int x = transformedPoints.coeff(0, i) - mapLimits_.minX;
		int y = transformedPoints.coeff(1, i) - mapLimits_.minY;
		UASSERT(x >= 0 && x < missCounter_.cols() && y >= 0 && y < missCounter_.rows());

		bool free = (i < nodeIt->localMap.numEmpty);
		if (free)
		{
			int& misses = missCounter_.coeffRef(y, x);
			misses -= 1;
			UASSERT(misses >= 0);
		}
		else
		{
			int& hits = hitCounter_.coeffRef(y, x);
			hits -= 1;
			UASSERT(hits >= 0);
		}
	}
	nodes_.erase(nodeIt);

	MapLimits newMapLimits;
	for (const Node& node : nodes_)
	{
		newMapLimits = MapLimits::unite(newMapLimits, node.transformedLocalMap->mapLimits);
	}
	createOrResizeMap(newMapLimits);
}

TemporaryColoredOccupancyGrid::OccupancyGrid TemporaryColoredOccupancyGrid::getOccupancyGrid() const
{
	MEASURE_BLOCK_TIME(TemporaryColoredOccupancyGrid__getOccupancyGrid);
	UASSERT(mapLimits_.valid());
	OccupancyGrid occupancyGrid;
	occupancyGrid.minY = mapLimits_.minY * cellSize_;
	occupancyGrid.minX = mapLimits_.minX * cellSize_;
	occupancyGrid.grid =
		OccupancyGrid::GridType::Constant(mapLimits_.height(), mapLimits_.width(), -1);
	for(int x = 0; x < missCounter_.cols(); ++x)
	{
		for(int y = 0; y < missCounter_.rows(); ++y)
		{
			int misses = missCounter_.coeff(y, x);
			int hits = hitCounter_.coeff(y, x);
			float value = misses * miss_ + hits * hit_;
			if(misses == 0 && hits == 0)
			{
				continue;
			}
			else if(value >= occupancyThr_)
			{
				occupancyGrid.grid.coeffRef(y, x) = 100;
			}
			else
			{
				occupancyGrid.grid.coeffRef(y, x) = 0;
			}
		}
	}
	return occupancyGrid;
}

TemporaryColoredOccupancyGrid::OccupancyGrid TemporaryColoredOccupancyGrid::getProbOccupancyGrid() const
{
	MEASURE_BLOCK_TIME(TemporaryColoredOccupancyGrid__getProbOccupancyGrid);
	UASSERT(mapLimits_.valid());
	OccupancyGrid occupancyGrid;
	occupancyGrid.minY = mapLimits_.minY * cellSize_;
	occupancyGrid.minX = mapLimits_.minX * cellSize_;
	occupancyGrid.grid =
		OccupancyGrid::GridType::Constant(mapLimits_.height(), mapLimits_.width(), -1);
	for(int x = 0; x < missCounter_.cols(); ++x)
	{
		for(int y = 0; y < missCounter_.rows(); ++y)
		{
			int misses = missCounter_.coeff(y, x);
			int hits = hitCounter_.coeff(y, x);
			float value = misses * miss_ + hits * hit_;
			if(misses == 0 && hits == 0)
			{
				continue;
			}
			else
			{
				occupancyGrid.grid.coeffRef(y, x) = probability(value) * 100;
			}
		}
	}
	return occupancyGrid;
}

TemporaryColoredOccupancyGrid::ColorGrid TemporaryColoredOccupancyGrid::getColorGrid() const
{
	MEASURE_BLOCK_TIME(ColoredOccupancyGrid__getColorGrid);
	UASSERT(mapLimits_.valid());
	ColorGrid colorGrid;
	colorGrid.minY = mapLimits_.minY * cellSize_;
	colorGrid.minX = mapLimits_.minX * cellSize_;
	colorGrid.grid = colors_;
	return colorGrid;
}

void TemporaryColoredOccupancyGrid::clear()
{
	for (Node& node : nodes_)
	{
		node.transformedLocalMap.reset();
	}
	mapLimits_ = MapLimits();
	missCounter_ = Eigen::MatrixXi();
	hitCounter_ = Eigen::MatrixXi();
	colors_ = Eigen::MatrixXi();
}

}
