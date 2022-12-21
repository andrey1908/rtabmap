#include <rtabmap/core/OccupancyGridBuilder.h>
#include <rtabmap/utilite/ULogger.h>

#include "time_measurer/time_measurer.h"

namespace rtabmap {

OccupancyGridBuilder::OccupancyGridBuilder(const ParametersMap& parameters) :
	cellSize_(Parameters::defaultGridCellSize()),
	missProb_(Parameters::defaultGridGlobalMissProb()),
	hitProb_(Parameters::defaultGridGlobalHitProb()),
	minClampingProb_(Parameters::defaultGridGlobalMinClampingProb()),
	maxClampingProb_(Parameters::defaultGridGlobalMaxClampingProb()),
	occupancyProbThr_(Parameters::defaultGridGlobalOccupancyThr()),
	temporarilyOccupiedCellColorRgb_(Parameters::defaultGridGlobalTemporarilyOccupiedCellColor()),
	showTemporarilyOccupiedCells_(Parameters::defaultGridGlobalShowTemporarilyOccupiedCells())
{
	parseParameters(parameters);
}

void OccupancyGridBuilder::parseParameters(const ParametersMap & parameters)
{
	Parameters::parse(parameters, Parameters::kGridCellSize(), cellSize_);
	Parameters::parse(parameters, Parameters::kGridGlobalMissProb(), missProb_);
	Parameters::parse(parameters, Parameters::kGridGlobalHitProb(), hitProb_);
	Parameters::parse(parameters, Parameters::kGridGlobalMinClampingProb(), minClampingProb_);
	Parameters::parse(parameters, Parameters::kGridGlobalMaxClampingProb(), maxClampingProb_);
	Parameters::parse(parameters, Parameters::kGridGlobalOccupancyThr(), occupancyProbThr_);
	Parameters::parse(parameters, Parameters::kGridGlobalTemporarilyOccupiedCellColor(), temporarilyOccupiedCellColorRgb_);
	Parameters::parse(parameters, Parameters::kGridGlobalShowTemporarilyOccupiedCells(), showTemporarilyOccupiedCells_);
	UASSERT(missProb_ > 0.0f && missProb_ <= 0.5f);
	UASSERT(hitProb_ >= 0.5f && hitProb_ < 1.0f);
	UASSERT(minClampingProb_ > 0.0f && minClampingProb_ < 1.0f);
	UASSERT(maxClampingProb_ > 0.0f && maxClampingProb_ < 1.0f);
	UASSERT(minClampingProb_ < maxClampingProb_);
	UASSERT(occupancyProbThr_ > 0.0f && occupancyProbThr_ < 1.0f);

	float missLogit = logodds(missProb_);
	float hitLogit = logodds(hitProb_);
	float minClampingLogit = logodds(minClampingProb_);
	float maxClampingLogit = logodds(maxClampingProb_);
	occupancyThr_ = probabilityToVlaue(occupancyProbThr_);
	updated_ = splitNum_ * 2;
	if (temporarilyOccupiedCellColorRgb_ >= 0)
	{
		temporarilyOccupiedCellColor_.setRgb(temporarilyOccupiedCellColorRgb_);
	}

	missUpdates_.clear();
	hitUpdates_.clear();
	probabilities_.clear();
	probabilitiesThr_.clear();
	for (int value = 0; value <= splitNum_ + 1; value++)
	{
		float prob = valueToProbability(value);
		if (value == 1 || value == splitNum_ + 1)
		{
			// prob = 0.0 or prob = 1.0
			missUpdates_.push_back(value);
			hitUpdates_.push_back(value);
		}
		else
		{
			float logit = logodds(prob);
			float missUpdate = logit + missLogit;
			if (missUpdate < minClampingLogit)
			{
				missUpdate = minClampingLogit;
			}
			float hitUpdate = logit + hitLogit;
			if (hitUpdate > maxClampingLogit)
			{
				hitUpdate = maxClampingLogit;
			}
			missUpdates_.push_back(probabilityToVlaue(probability(missUpdate)));
			hitUpdates_.push_back(probabilityToVlaue(probability(hitUpdate)));
		}
		if (value == 0)
		{
			// unknown
			probabilitiesThr_.push_back(-1);
			probabilities_.push_back(-1);
		}
		else
		{
			probabilities_.push_back(std::lround(prob * 100.0f));
			if (prob >= occupancyProbThr_)
			{
				probabilitiesThr_.push_back(100);
			}
			else
			{
				probabilitiesThr_.push_back(0);
			}
		}
	}
	for (int& updatedValue : missUpdates_)
	{
		updatedValue += updated_;
	}
	for (int& updatedValue : hitUpdates_)
	{
		updatedValue += updated_;
	}
}

void OccupancyGridBuilder::addLocalMap(int nodeId, std::shared_ptr<const LocalMap> localMap)
{
	UASSERT(localMap);
	UASSERT(nodeId >= 0);
	UASSERT(nodes_.empty() || nodes_.rbegin()->first < nodeId);
	Node node(localMap, std::nullopt);
	nodes_.emplace(nodeId, std::move(node));
}

void OccupancyGridBuilder::addLocalMap(int nodeId, std::shared_ptr<const LocalMap> localMap,
	const Transform& pose)
{
	MEASURE_BLOCK_TIME(OccupancyGridBuilder__addLocalMap__withPose);
	UASSERT(localMap);
	UASSERT(nodeId >= 0);
	UASSERT(nodes_.empty() || nodes_.rbegin()->first < nodeId);
	TransformedLocalMap transformedLocalMap = transformLocalMap(*localMap, pose);
	Node node(localMap, std::move(transformedLocalMap));
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
	cachedMap_ = MapType();
	cachedColors_ = ColorsType();
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
	static time_measurer::TimeMeasurer OccupancyGridBuilder__tryToUseCachedMap__fail(
		"OccupancyGridBuilder__tryToUseCachedMap__fail", true);
	static time_measurer::TimeMeasurer OccupancyGridBuilder__tryToUseCachedMap__success(
		"OccupancyGridBuilder__tryToUseCachedMap__success", true);
	OccupancyGridBuilder__tryToUseCachedMap__fail.StartMeasurement();
	OccupancyGridBuilder__tryToUseCachedMap__success.StartMeasurement();

	if (!checkIfCachedMapCanBeUsed(updatedPoses))
	{
		OccupancyGridBuilder__tryToUseCachedMap__fail.StopMeasurement();
		return -1;
	}

	useCachedMap();
	OccupancyGridBuilder__tryToUseCachedMap__success.StopMeasurement();
	return cachedPoses_.rbegin()->first;
}

void OccupancyGridBuilder::updatePoses(const std::map<int, Transform>& updatedPoses,
	int lastNodeIdForCachedMap /* -1 */)
{
	MEASURE_BLOCK_TIME(OccupancyGridBuilder__updatePoses);
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
		node.transformedLocalMap = transformLocalMap(*node.localMap, updatedPoseIt->second);
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
		mapLimits_ = newMapLimits;
		int height = newMapLimits.height();
		int width = newMapLimits.width();
		map_ = MapType::Constant(height, width, unknown_);
		colors_ = ColorsType::Constant(height, width, Color::missingColor.data());
	}
	else if(mapLimits_ != newMapLimits)
	{
		int dstStartY = std::max(mapLimits_.minY - newMapLimits.minY, 0);
		int dstStartX = std::max(mapLimits_.minX - newMapLimits.minX, 0);
		int srcStartY = std::max(newMapLimits.minY - mapLimits_.minY, 0);
		int srcStartX = std::max(newMapLimits.minX - mapLimits_.minX, 0);
		MapLimits intersection = MapLimits::intersect(mapLimits_, newMapLimits);
		int copyHeight = intersection.height();
		int copyWidth = intersection.width();
		UASSERT(copyHeight > 0 && copyWidth > 0);

		int height = newMapLimits.height();
		int width = newMapLimits.width();
		MapType newMap = MapType::Constant(height, width, unknown_);
		ColorsType newColors =
			ColorsType::Constant(height, width, Color::missingColor.data());

		newMap.block(dstStartY, dstStartX, copyHeight, copyWidth) =
			map_.block(srcStartY, srcStartX, copyHeight, copyWidth);
		newColors.block(dstStartY, dstStartX, copyHeight, copyWidth) =
			colors_.block(srcStartY, srcStartX, copyHeight, copyWidth);

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
	UASSERT(node.transformedLocalMap.has_value());
	temporarilyOccupiedCells_.clear();
	const Eigen::Matrix2Xi& transformedPoints = node.transformedLocalMap->points;
	for (int i = 0; i < transformedPoints.cols(); i++)
	{
		int y = transformedPoints.coeff(1, i) - mapLimits_.minY;
		int x = transformedPoints.coeff(0, i) - mapLimits_.minX;
		UASSERT(y >= 0 && x >= 0 && y < map_.rows() && x < map_.cols());

		int& value = map_.coeffRef(y, x);
		if (value >= updated_)
		{
			continue;
		}
		bool occupied = (i < node.localMap->numObstacles);
		if (occupied)
		{
			if (temporarilyOccupiedCellColor_ != Color::missingColor)
			{
				const Color& color = node.localMap->colors[i];
				if (color == temporarilyOccupiedCellColor_)
				{
					temporarilyOccupiedCells_.emplace_back(y, x);
					continue;
				}
			}
			value = hitUpdates_[value];
		}
		else
		{
			if (node.localMap->sensorBlindRange2dSqr != 0.0f &&
				value >= occupancyThr_)
			{
				float localX = node.localMap->points.coeff(0, i);
				float localY = node.localMap->points.coeff(1, i);
				float sensorX = localX - node.localMap->toSensor.translation().x();
				float sensorY = localY - node.localMap->toSensor.translation().y();
				if (sensorX * sensorX + sensorY * sensorY <=
					node.localMap->sensorBlindRange2dSqr)
				{
					continue;
				}
			}
			value = missUpdates_[value];
		}

		const Color& color = node.localMap->colors[i];
		if (!color.missing())
		{
			colors_.coeffRef(y, x) = color.rgb();
		}
	}
	for (int y = 0; y < map_.rows(); y++)
	{
		for (int x = 0; x < map_.cols(); x++)
		{
			if (map_.coeffRef(y, x) >= updated_)
			{
				map_.coeffRef(y, x) -= updated_;
			}
		}
	}
}

OccupancyGridBuilder::OccupancyGrid OccupancyGridBuilder::getOccupancyGrid() const
{
	if (!mapLimits_.valid())
	{
		return OccupancyGrid();
	}
	return getOccupancyGrid(mapLimits_);
}

OccupancyGridBuilder::OccupancyGrid OccupancyGridBuilder::getOccupancyGrid(
	const MapLimits& roi) const
{
	OccupancyGrid occupancyGrid;
	occupancyGrid.limits = roi;
	occupancyGrid.grid = OccupancyGrid::GridType::Constant(roi.height(), roi.width(), -1);
	MapLimits intersection = MapLimits::intersect(mapLimits_, roi);
	int height = intersection.height();
	int width = intersection.width();
	if (height == 0 || width == 0)
	{
		return occupancyGrid;
	}
	int dstStartY = std::max(mapLimits_.minY - roi.minY, 0);
	int dstStartX = std::max(mapLimits_.minX - roi.minX, 0);
	int srcStartY = std::max(roi.minY - mapLimits_.minY, 0);
	int srcStartX = std::max(roi.minX - mapLimits_.minX, 0);
	for(int y = 0; y < height; ++y)
	{
		for(int x = 0; x < width; ++x)
		{
			int value = map_.coeff(y + srcStartY, x + srcStartX);
			occupancyGrid.grid.coeffRef(y + dstStartY, x + dstStartX) =
				probabilitiesThr_[value];
		}
	}
	if (showTemporarilyOccupiedCells_)
	{
		for (const auto& coords : temporarilyOccupiedCells_)
		{
			int y = coords.first;
			int x = coords.second;
			y -= srcStartY;
			x -= srcStartX;
			if (y >= 0 && x >= 0 && y < height && x < width)
			{
				occupancyGrid.grid.coeffRef(y + dstStartY, x + dstStartX) = 100;
			}
		}
	}
	return occupancyGrid;
}

OccupancyGridBuilder::OccupancyGrid OccupancyGridBuilder::getProbOccupancyGrid() const
{
	if (!mapLimits_.valid())
	{
		return OccupancyGrid();
	}
	return getProbOccupancyGrid(mapLimits_);
}

OccupancyGridBuilder::OccupancyGrid OccupancyGridBuilder::getProbOccupancyGrid(
	const MapLimits& roi) const
{
	OccupancyGrid occupancyGrid;
	occupancyGrid.limits = roi;
	occupancyGrid.grid = OccupancyGrid::GridType::Constant(roi.height(), roi.width(), -1);
	MapLimits intersection = MapLimits::intersect(mapLimits_, roi);
	int height = intersection.height();
	int width = intersection.width();
	if (height == 0 || width == 0)
	{
		return occupancyGrid;
	}
	int dstStartY = std::max(mapLimits_.minY - roi.minY, 0);
	int dstStartX = std::max(mapLimits_.minX - roi.minX, 0);
	int srcStartY = std::max(roi.minY - mapLimits_.minY, 0);
	int srcStartX = std::max(roi.minX - mapLimits_.minX, 0);
	for(int y = 0; y < height; ++y)
	{
		for(int x = 0; x < width; ++x)
		{
			int value = map_.coeff(y + srcStartY, x + srcStartX);
			occupancyGrid.grid.coeffRef(y + dstStartY, x + dstStartX) =
				probabilities_[value];
		}
	}
	if (showTemporarilyOccupiedCells_)
	{
		for (const auto& coords : temporarilyOccupiedCells_)
		{
			int y = coords.first;
			int x = coords.second;
			y -= srcStartY;
			x -= srcStartX;
			if (y >= 0 && x >= 0 && y < height && x < width)
			{
				occupancyGrid.grid.coeffRef(y + dstStartY, x + dstStartX) = 100;
			}
		}
	}
	return occupancyGrid;
}

OccupancyGridBuilder::ColorGrid OccupancyGridBuilder::getColorGrid() const
{
	if (!mapLimits_.valid())
	{
		return ColorGrid();
	}
	return getColorGrid(mapLimits_);
}

OccupancyGridBuilder::ColorGrid OccupancyGridBuilder::getColorGrid(
	const MapLimits& roi) const
{
	ColorGrid colorGrid;
	colorGrid.limits = roi;
	colorGrid.grid = ColorGrid::GridType::Constant(roi.height(), roi.width(),
		Color::missingColor.data());
	MapLimits intersection = MapLimits::intersect(mapLimits_, roi);
	int height = intersection.height();
	int width = intersection.width();
	if (height == 0 || width == 0)
	{
		return colorGrid;
	}
	int dstStartY = std::max(mapLimits_.minY - roi.minY, 0);
	int dstStartX = std::max(mapLimits_.minX - roi.minX, 0);
	int srcStartY = std::max(roi.minY - mapLimits_.minY, 0);
	int srcStartX = std::max(roi.minX - mapLimits_.minX, 0);
	colorGrid.grid.block(dstStartY, dstStartX, height, width) =
		colors_.block(srcStartY, srcStartX, height, width);
	if (showTemporarilyOccupiedCells_)
	{
		for (const auto& coords : temporarilyOccupiedCells_)
		{
			int y = coords.first;
			int x = coords.second;
			y -= srcStartY;
			x -= srcStartX;
			if (y >= 0 && x >= 0 && y < height && x < width)
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
	map_ = MapType();
	colors_ = ColorsType();
	temporarilyOccupiedCells_.clear();
}

void OccupancyGridBuilder::reset()
{
	nodes_.clear();
	mapLimits_ = MapLimits();
	map_ = MapType();
	colors_ = ColorsType();
	temporarilyOccupiedCells_.clear();
}

}
