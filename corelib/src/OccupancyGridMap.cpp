#include <rtabmap/core/OccupancyGridMap.h>
#include <rtabmap/utilite/ULogger.h>

#include "time_measurer/time_measurer.h"

namespace rtabmap {

OccupancyGridMap::OccupancyGridMap(const ParametersMap& parameters) :
	cellSize_(Parameters::defaultGridCellSize())
{
	parseParameters(parameters);
}

void OccupancyGridMap::parseParameters(const ParametersMap& parameters)
{
	Parameters::parse(parameters, Parameters::kGridCellSize(), cellSize_);

	localMapBuilder_ = std::make_unique<LocalMapBuilder>(parameters);
	obstacleDilation_ = std::make_unique<ObstacleDilation>(parameters);
	occupancyGridBuilder_ = std::make_unique<OccupancyGridBuilder>(parameters);
	temporaryOccupancyGridBuilder_ =
		std::make_unique<TemporaryOccupancyGridBuilder>(parameters);
}

std::shared_ptr<const OccupancyGridMap::LocalMap> OccupancyGridMap::createLocalMap(
	const Signature & signature) const
{
	return localMapBuilder_->createLocalMap(signature);
}

void OccupancyGridMap::addLocalMap(int nodeId, std::shared_ptr<const LocalMap> localMap)
{
	localMapsWithoutObstacleDilation_.emplace(nodeId, localMap);
	std::shared_ptr<const LocalMap> dilatedLocalMap;
	if (obstacleDilation_->dilationSize() != 0.0f)
	{
		MEASURE_BLOCK_TIME(OccupancyGridMap__obstacleDilation);
		dilatedLocalMap = obstacleDilation_->dilate(*localMap);
	}
	else
	{
		dilatedLocalMap = localMap;
	}
	occupancyGridBuilder_->addLocalMap(nodeId, dilatedLocalMap);
}

void OccupancyGridMap::addLocalMap(int nodeId, std::shared_ptr<const LocalMap> localMap,
	const Transform& pose)
{
	localMapsWithoutObstacleDilation_.emplace(nodeId, localMap);
	std::shared_ptr<const LocalMap> dilatedLocalMap;
	if (obstacleDilation_->dilationSize() != 0.0f)
	{
		MEASURE_BLOCK_TIME(OccupancyGridMap__obstacleDilation);
		dilatedLocalMap = obstacleDilation_->dilate(*localMap);
	}
	else
	{
		dilatedLocalMap = localMap;
	}
	occupancyGridBuilder_->addLocalMap(nodeId, dilatedLocalMap, pose);
}

void OccupancyGridMap::addTemporaryLocalMap(std::shared_ptr<const LocalMap> localMap,
	const Transform& pose)
{
	std::shared_ptr<const LocalMap> dilatedLocalMap;
	if (obstacleDilation_->dilationSize() != 0.0f)
	{
		MEASURE_BLOCK_TIME(OccupancyGridMap__obstacleDilation);
		dilatedLocalMap = obstacleDilation_->dilate(*localMap);
	}
	else
	{
		dilatedLocalMap = localMap;
	}
	temporaryOccupancyGridBuilder_->addLocalMap(dilatedLocalMap, pose);
}

void OccupancyGridMap::cacheCurrentMap()
{
	occupancyGridBuilder_->cacheCurrentMap();
}

void OccupancyGridMap::updatePoses(const std::map<int, Transform>& updatedPoses,
		const std::list<Transform>& updatedTemporaryPoses,
		int lastNodeIdForCachedMap /* -1 */)
{
	occupancyGridBuilder_->updatePoses(updatedPoses, lastNodeIdForCachedMap);
	temporaryOccupancyGridBuilder_->updatePoses(updatedTemporaryPoses);
}

OccupancyGridMap::OccupancyGrid OccupancyGridMap::getOccupancyGrid() const
{
	MEASURE_BLOCK_TIME(OccupancyGridMap__getOccupancyGrid);
	MapLimits mapLimits = occupancyGridBuilder_->mapLimits();
	MapLimits temporaryMapLimits = temporaryOccupancyGridBuilder_->mapLimits();
	UASSERT(mapLimits.valid() || temporaryMapLimits.valid());
	if (!mapLimits.valid())
	{
		return temporaryOccupancyGridBuilder_->getOccupancyGrid();
	}
	if (!temporaryMapLimits.valid())
	{
		return occupancyGridBuilder_->getOccupancyGrid();
	}

	MapLimits combinedMapLimits =
		MapLimits::unite(mapLimits, temporaryMapLimits);
	OccupancyGrid occupancyGrid =
		occupancyGridBuilder_->getOccupancyGrid(combinedMapLimits);
	OccupancyGrid temporaryOccupancyGrid =
		temporaryOccupancyGridBuilder_->getOccupancyGrid();

	int dstStartY = temporaryOccupancyGrid.limits.minY() - occupancyGrid.limits.minY();
	int dstStartX = temporaryOccupancyGrid.limits.minX() - occupancyGrid.limits.minX();
	int height = temporaryOccupancyGrid.limits.height();
	int width = temporaryOccupancyGrid.limits.width();
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			char value = temporaryOccupancyGrid.grid.coeff(y, x);
			if (value != -1)
			{
				occupancyGrid.grid.coeffRef(y + dstStartY, x + dstStartX) = value;
			}
		}
	}
	return occupancyGrid;
}

OccupancyGridMap::OccupancyGrid OccupancyGridMap::getProbOccupancyGrid() const
{
	MEASURE_BLOCK_TIME(OccupancyGridMap__getProbOccupancyGrid);
	MapLimits mapLimits = occupancyGridBuilder_->mapLimits();
	MapLimits temporaryMapLimits = temporaryOccupancyGridBuilder_->mapLimits();
	UASSERT(mapLimits.valid() || temporaryMapLimits.valid());
	if (!mapLimits.valid())
	{
		return temporaryOccupancyGridBuilder_->getOccupancyGrid();
	}
	if (!temporaryMapLimits.valid())
	{
		return occupancyGridBuilder_->getOccupancyGrid();
	}

	MapLimits combinedMapLimits =
		MapLimits::unite(mapLimits, temporaryMapLimits);
	OccupancyGrid occupancyGrid =
		occupancyGridBuilder_->getProbOccupancyGrid(combinedMapLimits);
	OccupancyGrid temporaryOccupancyGrid =
		temporaryOccupancyGridBuilder_->getProbOccupancyGrid();

	int dstStartY = temporaryOccupancyGrid.limits.minY() - occupancyGrid.limits.minY();
	int dstStartX = temporaryOccupancyGrid.limits.minX() - occupancyGrid.limits.minX();
	int height = temporaryOccupancyGrid.limits.height();
	int width = temporaryOccupancyGrid.limits.width();
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			char value = temporaryOccupancyGrid.grid.coeff(y, x);
			if (value != -1)
			{
				occupancyGrid.grid.coeffRef(y + dstStartY, x + dstStartX) = value;
			}
		}
	}
	return occupancyGrid;
}

OccupancyGridMap::ColorGrid OccupancyGridMap::getColorGrid() const
{
	MEASURE_BLOCK_TIME(OccupancyGridMap__getColorGrid);
	MapLimits mapLimits = occupancyGridBuilder_->mapLimits();
	MapLimits temporaryMapLimits = temporaryOccupancyGridBuilder_->mapLimits();
	UASSERT(mapLimits.valid() || temporaryMapLimits.valid());
	if (!mapLimits.valid())
	{
		return temporaryOccupancyGridBuilder_->getColorGrid();
	}
	if (!temporaryMapLimits.valid())
	{
		return occupancyGridBuilder_->getColorGrid();
	}

	MapLimits combinedMapLimits =
		MapLimits::unite(mapLimits, temporaryMapLimits);
	ColorGrid colorGrid =
		occupancyGridBuilder_->getColorGrid(combinedMapLimits);
	ColorGrid temporaryColorGrid =
		temporaryOccupancyGridBuilder_->getColorGrid();

	int dstStartY = temporaryColorGrid.limits.minY() - colorGrid.limits.minY();
	int dstStartX = temporaryColorGrid.limits.minX() - colorGrid.limits.minX();
	int height = temporaryColorGrid.limits.height();
	int width = temporaryColorGrid.limits.width();
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			int value = temporaryColorGrid.grid.coeff(y, x);
			if (value != Color::missingColor.data())
			{
				colorGrid.grid.coeffRef(y + dstStartY, x + dstStartX) = value;
			}
		}
	}
	return colorGrid;
}

std::pair<float, float> OccupancyGridMap::getGridOrigin() const
{
	MapLimits mapLimits = MapLimits::unite(occupancyGridBuilder_->mapLimits(),
		temporaryOccupancyGridBuilder_->mapLimits());
	UASSERT(mapLimits.valid());
	float originX = mapLimits.minX() * cellSize_;
	float originY = mapLimits.minY() * cellSize_;
	return std::make_pair(originX, originY);
}

void OccupancyGridMap::resetAll()
{
	occupancyGridBuilder_->reset();
	temporaryOccupancyGridBuilder_->reset();
}

void OccupancyGridMap::resetTemporaryMap()
{
	temporaryOccupancyGridBuilder_->reset();
}

}
