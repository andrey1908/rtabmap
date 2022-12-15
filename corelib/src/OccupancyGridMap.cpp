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
	LocalMap localMap;
	localMap.LocalMapBuilder::LocalMap::operator=(localMapBuilder_.createLocalMap(signature));
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
	OccupancyGrid occupancyGrid = occupancyGridBuilder_.getOccupancyGrid();
	OccupancyGrid temporaryOccupancyGrid = temporaryOccupancyGridBuilder_.getOccupancyGrid();
	
	MapLimits occupancyGridML;
	if (occupancyGrid.minX != std::numeric_limits<int>::max())
	{
		occupancyGridML.minX = occupancyGrid.minX;
		occupancyGridML.minY = occupancyGrid.minY;
		occupancyGridML.maxX = occupancyGrid.minX + occupancyGrid.grid.cols();
		occupancyGridML.maxY = occupancyGrid.minY + occupancyGrid.grid.rows();
	}

	MapLimits temporaryOccupancyGridML;
	if (temporaryOccupancyGrid.minX != std::numeric_limits<int>::max())
	{
		temporaryOccupancyGridML.minX = temporaryOccupancyGrid.minX;
		temporaryOccupancyGridML.minY = temporaryOccupancyGrid.minY;
		temporaryOccupancyGridML.maxX = temporaryOccupancyGrid.minX + temporaryOccupancyGrid.grid.cols();
		temporaryOccupancyGridML.maxY = temporaryOccupancyGrid.minY + temporaryOccupancyGrid.grid.rows();
	}

	MapLimits combinedML = MapLimits::unite(occupancyGridML, temporaryOccupancyGridML);
	OccupancyGrid combined;
	combined.minX = combinedML.minX;
	combined.minY = combinedML.minY;
	combined.grid = OccupancyGrid::GridType::Constant(combinedML.height(), combinedML.width(),
		-1);
	if (occupancyGrid.minX != std::numeric_limits<int>::max())
	{
		int occupancyGridShiftX = occupancyGridML.minX - combinedML.minX;
		int occupancyGridShiftY = occupancyGridML.minY - combinedML.minY;
		combined.grid.block(occupancyGridShiftY, occupancyGridShiftX,
			occupancyGridML.height(), occupancyGridML.width()) = occupancyGrid.grid;
	}
	if (temporaryOccupancyGrid.minX != std::numeric_limits<int>::max())
	{
	int temporaryOccupancyGridShiftX = temporaryOccupancyGridML.minX - combinedML.minX;
	int temporaryOccupancyGridShiftY = temporaryOccupancyGridML.minY - combinedML.minY;
	combined.grid.block(temporaryOccupancyGridShiftY, temporaryOccupancyGridShiftX,
		temporaryOccupancyGridML.height(), temporaryOccupancyGridML.width()) =
		temporaryOccupancyGrid.grid;
	}

	return combined;
}

OccupancyGridMap::OccupancyGrid OccupancyGridMap::getProbOccupancyGrid() const
{
	MEASURE_BLOCK_TIME(OccupancyGrid__getOccupancyGrid);
	OccupancyGrid occupancyGrid = occupancyGridBuilder_.getProbOccupancyGrid();
	OccupancyGrid temporaryOccupancyGrid = temporaryOccupancyGridBuilder_.getProbOccupancyGrid();
	
	MapLimits occupancyGridML;
	if (occupancyGrid.minX != std::numeric_limits<int>::max())
	{
		occupancyGridML.minX = occupancyGrid.minX;
		occupancyGridML.minY = occupancyGrid.minY;
		occupancyGridML.maxX = occupancyGrid.minX + occupancyGrid.grid.cols();
		occupancyGridML.maxY = occupancyGrid.minY + occupancyGrid.grid.rows();
	}

	MapLimits temporaryOccupancyGridML;
	if (temporaryOccupancyGrid.minX != std::numeric_limits<int>::max())
	{
		temporaryOccupancyGridML.minX = temporaryOccupancyGrid.minX;
		temporaryOccupancyGridML.minY = temporaryOccupancyGrid.minY;
		temporaryOccupancyGridML.maxX = temporaryOccupancyGrid.minX + temporaryOccupancyGrid.grid.cols();
		temporaryOccupancyGridML.maxY = temporaryOccupancyGrid.minY + temporaryOccupancyGrid.grid.rows();
	}

	MapLimits combinedML = MapLimits::unite(occupancyGridML, temporaryOccupancyGridML);
	OccupancyGrid combined;
	combined.minX = combinedML.minX;
	combined.minY = combinedML.minY;
	combined.grid = OccupancyGrid::GridType::Constant(combinedML.height(), combinedML.width(),
		-1);
	if (occupancyGrid.minX != std::numeric_limits<int>::max())
	{
		int occupancyGridShiftX = occupancyGridML.minX - combinedML.minX;
		int occupancyGridShiftY = occupancyGridML.minY - combinedML.minY;
		combined.grid.block(occupancyGridShiftY, occupancyGridShiftX,
			occupancyGridML.height(), occupancyGridML.width()) = occupancyGrid.grid;
	}
	if (temporaryOccupancyGrid.minX != std::numeric_limits<int>::max())
	{
		int temporaryOccupancyGridShiftX = temporaryOccupancyGridML.minX - combinedML.minX;
		int temporaryOccupancyGridShiftY = temporaryOccupancyGridML.minY - combinedML.minY;
		combined.grid.block(temporaryOccupancyGridShiftY, temporaryOccupancyGridShiftX,
			temporaryOccupancyGridML.height(), temporaryOccupancyGridML.width()) =
			temporaryOccupancyGrid.grid;
	}

	return combined;
}

OccupancyGridMap::ColorGrid OccupancyGridMap::getColorGrid() const
{
	MEASURE_BLOCK_TIME(OccupancyGrid__getOccupancyGrid);
	ColorGrid colorGrid = occupancyGridBuilder_.getColorGrid();
	ColorGrid temporaryColorGrid = temporaryOccupancyGridBuilder_.getColorGrid();
	
	MapLimits colorGridML;
	if (colorGrid.minX != std::numeric_limits<int>::max())
	{
		colorGridML.minX = colorGrid.minX;
		colorGridML.minY = colorGrid.minY;
		colorGridML.maxX = colorGrid.minX + colorGrid.grid.cols();
		colorGridML.maxY = colorGrid.minY + colorGrid.grid.rows();
	}

	MapLimits temporaryColorGridML;
	if (temporaryColorGrid.minX != std::numeric_limits<int>::max())
	{
		temporaryColorGridML.minX = temporaryColorGrid.minX;
		temporaryColorGridML.minY = temporaryColorGrid.minY;
		temporaryColorGridML.maxX = temporaryColorGrid.minX + temporaryColorGrid.grid.cols();
		temporaryColorGridML.maxY = temporaryColorGrid.minY + temporaryColorGrid.grid.rows();
	}

	MapLimits combinedML = MapLimits::unite(colorGridML, temporaryColorGridML);
	ColorGrid combined;
	combined.minX = combinedML.minX;
	combined.minY = combinedML.minY;
	combined.grid = ColorGrid::GridType::Constant(combinedML.height(), combinedML.width(),
		-1);
	if (colorGrid.minX != std::numeric_limits<int>::max())
	{
		int colorGridShiftX = colorGridML.minX - combinedML.minX;
		int colorGridShiftY = colorGridML.minY - combinedML.minY;
		combined.grid.block(colorGridShiftY, colorGridShiftX,
			colorGridML.height(), colorGridML.width()) = colorGrid.grid;
	}
	if (temporaryColorGrid.minX != std::numeric_limits<int>::max())
	{
		int temporaryColorGridShiftX = temporaryColorGridML.minX - combinedML.minX;
		int temporaryColorGridShiftY = temporaryColorGridML.minY - combinedML.minY;
		combined.grid.block(temporaryColorGridShiftY, temporaryColorGridShiftX,
			temporaryColorGridML.height(), temporaryColorGridML.width()) =
			temporaryColorGrid.grid;
	}

	return combined;
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
