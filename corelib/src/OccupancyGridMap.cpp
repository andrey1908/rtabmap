#include <rtabmap/core/OccupancyGridMap.h>
#include <rtabmap/utilite/ULogger.h>

#include "time_measurer/time_measurer.h"

namespace rtabmap {

OccupancyGridMap::OccupancyGridMap(const ParametersMap& parameters) :
	cellSize_(Parameters::defaultGridCellSize()),
	sensorBlindRange2d_(Parameters::defaultGridSensorBlindRange2d()),
	localMapBuilder_(),
	occupancyGridBuilder_(),
	temporaryOccupancyGridBuilder_()
{
	parseParameters(parameters);
}

void OccupancyGridMap::parseParameters(const ParametersMap& parameters)
{
	Parameters::parse(parameters, Parameters::kGridCellSize(), cellSize_);
	Parameters::parse(parameters, Parameters::kGridSensorBlindRange2d(), sensorBlindRange2d_);

	sensorBlindRange2dSqr_ = sensorBlindRange2d_ * sensorBlindRange2d_;
	localMapBuilder_.parseParameters(parameters);
	occupancyGridBuilder_.parseParameters(parameters);
	temporaryOccupancyGridBuilder_.parseParameters(parameters);
}

OccupancyGridMap::LocalMap OccupancyGridMap::createLocalMap(const Signature & signature) const
{
	LocalMap localMap = localMapBuilder_.createLocalMap(signature);
	localMap.sensorBlindRange2dSqr = sensorBlindRange2dSqr_;
	localMap.toSensor = signature.sensorData().laserScan().localTransform();
	return localMap;
}

void OccupancyGridMap::addLocalMap(int nodeId, LocalMap localMap)
{
	occupancyGridBuilder_.addLocalMap(nodeId, std::move(localMap));
}

void OccupancyGridMap::addLocalMap(int nodeId, LocalMap localMap, const Transform& pose)
{
	occupancyGridBuilder_.addLocalMap(nodeId, std::move(localMap), pose);
}

void OccupancyGridMap::addTemporaryLocalMap(LocalMap localMap, const Transform& pose)
{
	temporaryOccupancyGridBuilder_.addLocalMap(std::move(localMap), pose);
}

void OccupancyGridMap::updatePoses(const std::map<int, Transform>& updatedPoses,
		const std::list<Transform>& updatedTemporaryPoses,
		int lastNodeIdForCachedMap /* -1 */)
{
	occupancyGridBuilder_.updatePoses(updatedPoses, lastNodeIdForCachedMap);
	temporaryOccupancyGridBuilder_.updatePoses(updatedTemporaryPoses);
}

OccupancyGridMap::OccupancyGrid OccupancyGridMap::getOccupancyGrid() const
{
	MEASURE_BLOCK_TIME(OccupancyGrid__getOccupancyGrid);
	MapLimits occupancyGridMapLimits = occupancyGridBuilder_.mapLimits();
	MapLimits temporaryOccupancyGridMapLimits = temporaryOccupancyGridBuilder_.mapLimits();
	UASSERT(occupancyGridMapLimits.valid() || temporaryOccupancyGridMapLimits.valid());
	if (!occupancyGridMapLimits.valid())
	{
		return temporaryOccupancyGridBuilder_.getOccupancyGrid();
	}
	if (!temporaryOccupancyGridMapLimits.valid())
	{
		return occupancyGridBuilder_.getOccupancyGrid();
	}

	MapLimits combinedMapLimits =
		MapLimits::unite(occupancyGridMapLimits, temporaryOccupancyGridMapLimits);
	OccupancyGrid occupancyGrid = occupancyGridBuilder_.getOccupancyGrid(combinedMapLimits);
	OccupancyGrid temporaryOccupancyGrid =
		temporaryOccupancyGridBuilder_.getOccupancyGrid();
	
	int dstStartX = temporaryOccupancyGrid.limits.minX - occupancyGrid.limits.minX;
	int dstStartY = temporaryOccupancyGrid.limits.minY - occupancyGrid.limits.minY;
	int width = temporaryOccupancyGrid.limits.width();
	int height = temporaryOccupancyGrid.limits.height();
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			char value = temporaryOccupancyGrid.grid.coeff(y, x);
			if (value != (char)(-1))
			{
				occupancyGrid.grid.coeffRef(y + dstStartY, x + dstStartX) = value;
			}
		}
	}
	return occupancyGrid;
}

OccupancyGridMap::OccupancyGrid OccupancyGridMap::getProbOccupancyGrid() const
{
	MEASURE_BLOCK_TIME(OccupancyGrid__getProbOccupancyGrid);
	MapLimits occupancyGridMapLimits = occupancyGridBuilder_.mapLimits();
	MapLimits temporaryOccupancyGridMapLimits = temporaryOccupancyGridBuilder_.mapLimits();
	UASSERT(occupancyGridMapLimits.valid() || temporaryOccupancyGridMapLimits.valid());
	if (!occupancyGridMapLimits.valid())
	{
		return temporaryOccupancyGridBuilder_.getProbOccupancyGrid();
	}
	if (!temporaryOccupancyGridMapLimits.valid())
	{
		return occupancyGridBuilder_.getProbOccupancyGrid();
	}

	MapLimits combinedMapLimits =
		MapLimits::unite(occupancyGridMapLimits, temporaryOccupancyGridMapLimits);
	OccupancyGrid occupancyGrid = occupancyGridBuilder_.getProbOccupancyGrid(combinedMapLimits);
	OccupancyGrid temporaryOccupancyGrid =
		temporaryOccupancyGridBuilder_.getProbOccupancyGrid();
	
	int dstStartX = temporaryOccupancyGrid.limits.minX - occupancyGrid.limits.minX;
	int dstStartY = temporaryOccupancyGrid.limits.minY - occupancyGrid.limits.minY;
	int width = temporaryOccupancyGrid.limits.width();
	int height = temporaryOccupancyGrid.limits.height();
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			char value = temporaryOccupancyGrid.grid.coeff(y, x);
			if (value != (char)(-1))
			{
				occupancyGrid.grid.coeffRef(y + dstStartY, x + dstStartX) = value;
			}
		}
	}
	return occupancyGrid;
}

OccupancyGridMap::ColorGrid OccupancyGridMap::getColorGrid() const
{
	MEASURE_BLOCK_TIME(OccupancyGrid__getOccupancyGrid);
	MapLimits occupancyGridMapLimits = occupancyGridBuilder_.mapLimits();
	MapLimits temporaryOccupancyGridMapLimits = temporaryOccupancyGridBuilder_.mapLimits();
	UASSERT(occupancyGridMapLimits.valid() || temporaryOccupancyGridMapLimits.valid());
	if (!occupancyGridMapLimits.valid())
	{
		return temporaryOccupancyGridBuilder_.getColorGrid();
	}
	if (!temporaryOccupancyGridMapLimits.valid())
	{
		return occupancyGridBuilder_.getColorGrid();
	}

	MapLimits combinedMapLimits =
		MapLimits::unite(occupancyGridMapLimits, temporaryOccupancyGridMapLimits);
	ColorGrid colorGrid = occupancyGridBuilder_.getColorGrid(combinedMapLimits);
	ColorGrid temporaryColorGrid =
		temporaryOccupancyGridBuilder_.getColorGrid();
	
	int dstStartX = temporaryColorGrid.limits.minX - colorGrid.limits.minX;
	int dstStartY = temporaryColorGrid.limits.minY - colorGrid.limits.minY;
	int width = temporaryColorGrid.limits.width();
	int height = temporaryColorGrid.limits.height();
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
	MapLimits mapLimits = MapLimits::unite(occupancyGridBuilder_.mapLimits(),
		temporaryOccupancyGridBuilder_.mapLimits());
	UASSERT(mapLimits.valid());
	float originX = mapLimits.minX * cellSize_;
	float originY = mapLimits.minY * cellSize_;
	return std::make_pair(originX, originY);
}

void OccupancyGridMap::resetAll()
{
	occupancyGridBuilder_ = OccupancyGridBuilder();
	temporaryOccupancyGridBuilder_ = TemporaryOccupancyGridBuilder();
}

void OccupancyGridMap::resetTemporaryMap()
{
	temporaryOccupancyGridBuilder_ = TemporaryOccupancyGridBuilder();
}

}
