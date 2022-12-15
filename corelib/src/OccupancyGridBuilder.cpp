#include <rtabmap/core/OccupancyGridBuilder.h>
#include <rtabmap/utilite/ULogger.h>

#include "time_measurer/time_measurer.h"

namespace rtabmap {

OccupancyGridBuilder::OccupancyGridBuilder(const ParametersMap& parameters) :
	cellSize_(Parameters::defaultGridCellSize()),
	missProb_(Parameters::defaultGridGlobalProbMiss()),
	hitProb_(Parameters::defaultGridGlobalProbHit()),
	minClampingProb_(Parameters::defaultGridGlobalMinClampingProb()),
	maxClampingProb_(Parameters::defaultGridGlobalMaxClampingProb()),
	occupancyProbThr_(Parameters::defaultGridGlobalOccupancyProbThr()),
	temporarilyOccupiedCellColorRgb_(Parameters::defaultGridGlobalTemporarilyOccupiedCellColor()),
	showTemporarilyOccupiedCells_(Parameters::defaultGridGlobalShowTemporarilyOccupiedCells())
{
	parseParameters(parameters);
}

void OccupancyGridBuilder::parseParameters(const ParametersMap & parameters)
{
	Parameters::parse(parameters, Parameters::kGridCellSize(), cellSize_);
	Parameters::parse(parameters, Parameters::kGridGlobalProbMiss(), missProb_);
	Parameters::parse(parameters, Parameters::kGridGlobalProbHit(), hitProb_);
	Parameters::parse(parameters, Parameters::kGridGlobalMinClampingProb(), minClampingProb_);
	Parameters::parse(parameters, Parameters::kGridGlobalMaxClampingProb(), maxClampingProb_);
	Parameters::parse(parameters, Parameters::kGridGlobalOccupancyProbThr(), occupancyProbThr_);
	Parameters::parse(parameters, Parameters::kGridGlobalTemporarilyOccupiedCellColor(), temporarilyOccupiedCellColorRgb_);
	Parameters::parse(parameters, Parameters::kGridGlobalShowTemporarilyOccupiedCells(), showTemporarilyOccupiedCells_);
	UASSERT(missProb_ > 0.0f && missProb_ <= 0.5f);
	UASSERT(hitProb_ >= 0.5f && hitProb_ < 1.0f);
	UASSERT(minClampingProb_ > 0.0f && minClampingProb_ < 1.0f);
	UASSERT(maxClampingProb_ > 0.0f && maxClampingProb_ < 1.0f);
	UASSERT(minClampingProb_ < maxClampingProb_);
	UASSERT(occupancyProbThr_ > 0.0f && occupancyProbThr_ < 1.0f);

	miss_ = logodds(missProb_);
	hit_ = logodds(hitProb_);
	minClamping_ = logodds(minClampingProb_);
	maxClamping_ = logodds(maxClampingProb_);
	occupancyThr_ = logodds(occupancyProbThr_);
	unknown_ = maxClamping_ + 1.0f;
	if (temporarilyOccupiedCellColorRgb_ >= 0)
	{
		temporarilyOccupiedCellColor_.setRgb(temporarilyOccupiedCellColorRgb_);
	}
}

void OccupancyGridBuilder::addLocalMap(int nodeId, LocalMap localMap)
{
	UASSERT(nodeId >= 0);
	UASSERT(nodes_.empty() || nodes_.rbegin()->first < nodeId);
	Node node(std::move(localMap), std::nullopt);
	nodes_.emplace(nodeId, std::move(node));
}

void OccupancyGridBuilder::addLocalMap(int nodeId, LocalMap localMap, const Transform& pose)
{
	UASSERT(nodeId >= 0);
	UASSERT(nodes_.empty() || nodes_.rbegin()->first < nodeId);
	TransformedLocalMap transformedLocalMap = transformLocalMap(localMap, pose);
	Node node(std::move(localMap), std::move(transformedLocalMap));
	auto newNodeIt = nodes_.emplace(nodeId, std::move(node)).first;
	MapLimits newMapLimits = MapLimits::unite(mapLimits_,
		newNodeIt->second.transformedLocalMap->mapLimits);
	if (mapLimits_ != newMapLimits)
	{
		createOrResizeMap(newMapLimits);
	}
	deployLocalMap(newNodeIt->second);
}

void OccupancyGridBuilder::cacheCurrentMap()
{
	cachedPoses_.clear();
	cachedMapLimits_ = MapLimits();
	cachedMap_ = Eigen::MatrixXf();
	cachedColors_ = Eigen::MatrixXi();
	cachedTemporarilyOccupiedCells_.clear();
	if(!mapLimits_.valid())
	{
		return;
	}

	for (const auto& entry : nodes_)
	{
		int nodeId = entry.first;
		const Node& node = entry.second;
		if (!node.transformedLocalMap.has_value())
		{
			continue;
		}
		cachedPoses_.emplace(nodeId, node.transformedLocalMap->pose);
	}
	cachedMapLimits_ = mapLimits_;
	cachedMap_ = map_;
	cachedColors_ = colors_;
	cachedTemporarilyOccupiedCells_ = temporarilyOccupiedCells_;
}

bool OccupancyGridBuilder::checkIfCachedMapCanBeUsed(
	const std::map<int, Transform>& updatedPoses)
{
	if (!cachedMapLimits_.valid())
	{
		return false;
	}
	auto updatedPoseIt = updatedPoses.begin();
	for (const auto& cachedPose : cachedPoses_)
	{
		if (updatedPoseIt == updatedPoses.end())
		{
			return false;
		}
		if (cachedPose != *updatedPoseIt)
		{
			return false;
		}
		++updatedPoseIt;
	}
	return true;
}

void OccupancyGridBuilder::useCachedMap()
{
	clear();
	auto nodeIt = nodes_.begin();
	for (const auto& cachedPose : cachedPoses_)
	{
		UASSERT(nodeIt != nodes_.end());
		while (nodeIt->first != cachedPose.first)
		{
			++nodeIt;
			UASSERT(nodeIt != nodes_.end());
		}
		nodeIt->second.transformedLocalMap = TransformedLocalMap();
		nodeIt->second.transformedLocalMap->pose = cachedPose.second;
		++nodeIt;
	}
	mapLimits_ = cachedMapLimits_;
	map_ = cachedMap_;
	colors_ = cachedColors_;
	temporarilyOccupiedCells_ = cachedTemporarilyOccupiedCells_;
}

int OccupancyGridBuilder::tryToUseCachedMap(const std::map<int, Transform>& updatedPoses)
{
	static time_measurer::TimeMeasurer OccupancyGrid__tryToUseCachedMap__fail("OccupancyGrid__tryToUseCachedMap__fail", true);
	static time_measurer::TimeMeasurer OccupancyGrid__tryToUseCachedMap__success("OccupancyGrid__tryToUseCachedMap__success", true);
	OccupancyGrid__tryToUseCachedMap__fail.StartMeasurement();
	OccupancyGrid__tryToUseCachedMap__success.StartMeasurement();

	if (!checkIfCachedMapCanBeUsed(updatedPoses))
	{
		OccupancyGrid__tryToUseCachedMap__fail.StopMeasurement();
		return -1;
	}

	useCachedMap();
	OccupancyGrid__tryToUseCachedMap__success.StopMeasurement();
	return cachedPoses_.rbegin()->first;
}

void OccupancyGridBuilder::updatePoses(const std::map<int, Transform>& updatedPoses,
	int lastNodeIdForCachedMap /* -1 */)
{
	MEASURE_BLOCK_TIME(ColoredOccupancyGrid__updatePoses);
	clear();
	int lastNodeIdFromCache = tryToUseCachedMap(updatedPoses);

	MapLimits newMapLimits = mapLimits_;
	std::list<int> nodeIdsToDeploy;
	auto nodeIt = nodes_.lower_bound(lastNodeIdFromCache + 1);
	for(auto updatedPoseIt = updatedPoses.lower_bound(lastNodeIdFromCache + 1);
		updatedPoseIt != updatedPoses.end(); ++updatedPoseIt)
	{
		UASSERT(nodeIt != nodes_.end());
		while (nodeIt->first != updatedPoseIt->first)
		{
			++nodeIt;
			UASSERT(nodeIt != nodes_.end());
		}
		int nodeId = nodeIt->first;
		Node& node = nodeIt->second;
		node.transformedLocalMap = transformLocalMap(node.localMap, updatedPoseIt->second);
		newMapLimits = MapLimits::unite(newMapLimits, node.transformedLocalMap->mapLimits);
		nodeIdsToDeploy.push_back(nodeId);

		if (nodeId == lastNodeIdForCachedMap)
		{
			createOrResizeMap(newMapLimits);
			for(auto nodeIdToDeploy : nodeIdsToDeploy)
			{
				deployLocalMap(nodeIdToDeploy);
			}
			cacheCurrentMap();
			nodeIdsToDeploy.clear();
		}

		++nodeIt;
	}
	if (newMapLimits.valid() && nodeIdsToDeploy.size())
	{
		createOrResizeMap(newMapLimits);
		for(auto nodeIdToDeploy : nodeIdsToDeploy)
		{
			deployLocalMap(nodeIdToDeploy);
		}
	}
}

OccupancyGridBuilder::TransformedLocalMap OccupancyGridBuilder::transformLocalMap(
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

void OccupancyGridBuilder::createOrResizeMap(const MapLimits& newMapLimits)
{
	UASSERT(newMapLimits.valid());
	if(!mapLimits_.valid())
	{
		MEASURE_BLOCK_TIME(ColoredOccupancyGrid__createOrResizeMap__createMap);
		mapLimits_ = newMapLimits;
		int height = newMapLimits.height();
		int width = newMapLimits.width();
		map_ = Eigen::MatrixXf::Constant(height, width, unknown_);
		colors_ = Eigen::MatrixXi::Constant(height, width, Color::missingColor.data());
	}
	else if(mapLimits_ != newMapLimits)
	{
		MEASURE_BLOCK_TIME(ColoredOccupancyGrid__createOrResizeMap__resizeMap);
		int dstShiftX = std::max(mapLimits_.minX - newMapLimits.minX, 0);
		int dstShiftY = std::max(mapLimits_.minY - newMapLimits.minY, 0);
		int srcShiftX = std::max(newMapLimits.minX - mapLimits_.minX, 0);
		int srcShiftY = std::max(newMapLimits.minY - mapLimits_.minY, 0);
		MapLimits intersection = MapLimits::intersect(mapLimits_, newMapLimits);
		int copyWidth = intersection.width();
		int copyHeight = intersection.height();
		UASSERT(copyWidth > 0 && copyHeight > 0);

		int height = newMapLimits.height();
		int width = newMapLimits.width();
		Eigen::MatrixXf newMap =
			Eigen::MatrixXf::Constant(height, width, unknown_);
		Eigen::MatrixXi newColors =
			Eigen::MatrixXi::Constant(height, width, Color::missingColor.data());

		newMap.block(dstShiftY, dstShiftX, copyHeight, copyWidth) =
			map_.block(srcShiftY, srcShiftX, copyHeight, copyWidth);
		newColors.block(dstShiftY, dstShiftX, copyHeight, copyWidth) =
			colors_.block(srcShiftY, srcShiftX, copyHeight, copyWidth);

		mapLimits_ = newMapLimits;
		map_ = std::move(newMap);
		colors_ = std::move(newColors);
	}
}

void OccupancyGridBuilder::deployLocalMap(int nodeId)
{
	deployLocalMap(nodes_.at(nodeId));
}

void OccupancyGridBuilder::deployLocalMap(const Node& node)
{
	MEASURE_BLOCK_TIME(ColoredOccupancyGrid__deployLocalMap);
	UASSERT(node.transformedLocalMap.has_value());
	temporarilyOccupiedCells_.clear();
	const Eigen::Matrix2Xi& transformedPoints = node.transformedLocalMap->points;
	for (int i = 0; i < transformedPoints.cols(); i++)
	{
		int x = transformedPoints.coeff(0, i) - mapLimits_.minX;
		int y = transformedPoints.coeff(1, i) - mapLimits_.minY;
		UASSERT(x >= 0 && x < map_.cols() && y >= 0 && y < map_.rows());

		bool free = (i < node.localMap.numEmpty);
		if (free)
		{
			float& value = map_.coeffRef(y, x);
			if (node.localMap.sensorBlindRange2dSqr != 0.0f &&
				value != unknown_ && value >= occupancyThr_)
			{
				float localX = node.localMap.points.coeff(0, i);
				float localY = node.localMap.points.coeff(1, i);
				float sensorX = localX - node.localMap.toSensor.translation().x();
				float sensorY = localY - node.localMap.toSensor.translation().y();
				if (sensorX * sensorX + sensorY * sensorY <= node.localMap.sensorBlindRange2dSqr)
				{
					continue;
				}
			}
			if (value == unknown_)
			{
				value = 0.0f;
			}
			value += miss_;
			if (value < minClamping_)
			{
				value = minClamping_;
			}
		}
		else
		{
			if (temporarilyOccupiedCellColor_ != Color::missingColor)
			{
				const Color& color = node.localMap.colors[i];
				if (color == temporarilyOccupiedCellColor_)
				{
					temporarilyOccupiedCells_.emplace_back(x, y);
					continue;
				}
			}

			float& value = map_.coeffRef(y, x);
			if (value == unknown_)
			{
				value = 0.0f;
			}
			value += hit_;
			if (value > maxClamping_)
			{
				value = maxClamping_;
			}
		}

		Color localMapColor = node.localMap.colors[i];
		if (!localMapColor.missing())
		{
			colors_.coeffRef(y, x) = localMapColor.rgb();
		}
	}
}

OccupancyGridBuilder::OccupancyGrid OccupancyGridBuilder::getOccupancyGrid() const
{
	return getOccupancyGrid(mapLimits_);
}

OccupancyGridBuilder::OccupancyGrid OccupancyGridBuilder::getOccupancyGrid(
	const MapLimits& roi) const
{
	MEASURE_BLOCK_TIME(ColoredOccupancyGrid__getOccupancyGrid);
	OccupancyGrid occupancyGrid;
	if (!mapLimits_.valid())
	{
		return occupancyGrid;
	}
	occupancyGrid.limits = roi;
	occupancyGrid.grid = OccupancyGrid::GridType::Constant(roi.height(), roi.width(), -1);
	MapLimits intersection = MapLimits::intersect(mapLimits_, roi);
	int height = intersection.height();
	int width = intersection.width();
	if (height == 0 || width == 0)
	{
		return occupancyGrid;
	}
	int dstStartX = std::max(mapLimits_.minX - roi.minX, 0);
	int dstStartY = std::max(mapLimits_.minY - roi.minY, 0);
	int srcStartX = std::max(roi.minX - mapLimits_.minX, 0);
	int srcStartY = std::max(roi.minY - mapLimits_.minY, 0);
	for(int x = 0; x < width; ++x)
	{
		for(int y = 0; y < height; ++y)
		{
			float value = map_.coeff(y + srcStartY, x + srcStartX);
			if(value == unknown_)
			{
				occupancyGrid.grid.coeffRef(y + dstStartY, x + dstStartX) = -1;
			}
			else if(value >= occupancyThr_)
			{
				occupancyGrid.grid.coeffRef(y + dstStartY, x + dstStartX) = 100;
			}
			else
			{
				occupancyGrid.grid.coeffRef(y + dstStartY, x + dstStartX) = 0;
			}
		}
	}
	if (showTemporarilyOccupiedCells_)
	{
		for (const auto& pair : temporarilyOccupiedCells_)
		{
			int x = pair.first;
			int y = pair.second;
			x -= srcStartX;
			y -= srcStartY;
			if (x >= 0 && y >= 0 && x < width && y < height)
			{
				occupancyGrid.grid.coeffRef(y + dstStartY, x + dstStartX) = 100;
			}
		}
	}
	return occupancyGrid;
}

OccupancyGridBuilder::OccupancyGrid OccupancyGridBuilder::getProbOccupancyGrid() const
{
	return getProbOccupancyGrid(mapLimits_);
}

OccupancyGridBuilder::OccupancyGrid OccupancyGridBuilder::getProbOccupancyGrid(
	const MapLimits& roi) const
{
	MEASURE_BLOCK_TIME(ColoredOccupancyGrid__getProbOccupancyGrid);
	OccupancyGrid occupancyGrid;
	if (!mapLimits_.valid())
	{
		return occupancyGrid;
	}
	occupancyGrid.limits = roi;
	occupancyGrid.grid = OccupancyGrid::GridType::Constant(roi.height(), roi.width(), -1);
	MapLimits intersection = MapLimits::intersect(mapLimits_, roi);
	int height = intersection.height();
	int width = intersection.width();
	if (height == 0 || width == 0)
	{
		return occupancyGrid;
	}
	int dstStartX = std::max(mapLimits_.minX - roi.minX, 0);
	int dstStartY = std::max(mapLimits_.minY - roi.minY, 0);
	int srcStartX = std::max(roi.minX - mapLimits_.minX, 0);
	int srcStartY = std::max(roi.minY - mapLimits_.minY, 0);
	for(int x = 0; x < width; ++x)
	{
		for(int y = 0; y < height; ++y)
		{
			float value = map_.coeff(y + srcStartY, x + srcStartX);
			if(value == unknown_)
			{
				occupancyGrid.grid.coeffRef(y + dstStartY, x + dstStartX) = -1;
			}
			else
			{
				occupancyGrid.grid.coeffRef(y + dstStartY, x + dstStartX) =
					probability(value) * 100;
			}
		}
	}
	if (showTemporarilyOccupiedCells_)
	{
		for (const auto& pair : temporarilyOccupiedCells_)
		{
			int x = pair.first;
			int y = pair.second;
			x -= srcStartX;
			y -= srcStartY;
			if (x >= 0 && y >= 0 && x < width && y < height)
			{
				occupancyGrid.grid.coeffRef(y + dstStartY, x + dstStartX) = 100;
			}
		}
	}
	return occupancyGrid;
}

OccupancyGridBuilder::ColorGrid OccupancyGridBuilder::getColorGrid() const
{
	return getColorGrid(mapLimits_);
}

OccupancyGridBuilder::ColorGrid OccupancyGridBuilder::getColorGrid(
	const MapLimits& roi) const
{
	MEASURE_BLOCK_TIME(ColoredOccupancyGrid__getColorGrid);
	ColorGrid colorGrid;
	if (!mapLimits_.valid())
	{
		return colorGrid;
	}
	colorGrid.limits = roi;
	colorGrid.grid = Eigen::MatrixXi::Constant(roi.height(), roi.width(),
		Color::missingColor.data());
	MapLimits intersection = MapLimits::intersect(mapLimits_, roi);
	int height = intersection.height();
	int width = intersection.width();
	if (height == 0 || width == 0)
	{
		return colorGrid;
	}
	int dstStartX = std::max(mapLimits_.minX - roi.minX, 0);
	int dstStartY = std::max(mapLimits_.minY - roi.minY, 0);
	int srcStartX = std::max(roi.minX - mapLimits_.minX, 0);
	int srcStartY = std::max(roi.minY - mapLimits_.minY, 0);
	colorGrid.grid.block(dstStartY, dstStartX, height, width) =
		colors_.block(srcStartY, srcStartX, height, width);
	if (showTemporarilyOccupiedCells_)
	{
		for (const auto& pair : temporarilyOccupiedCells_)
		{
			int x = pair.first;
			int y = pair.second;
			x -= srcStartX;
			y -= srcStartY;
			if (x >= 0 && y >= 0 && x < width && y < height)
			{
				colorGrid.grid.coeffRef(y + dstStartY, x + dstStartX) =
					temporarilyOccupiedCellColor_.rgb();
			}
		}
	}
	return colorGrid;
}

void OccupancyGridBuilder::clear()
{
	for (auto& entry : nodes_)
	{
		entry.second.transformedLocalMap.reset();
	}
	mapLimits_ = MapLimits();
	map_ = Eigen::MatrixXf();
	colors_ = Eigen::MatrixXi();
	temporarilyOccupiedCells_.clear();
}

}
