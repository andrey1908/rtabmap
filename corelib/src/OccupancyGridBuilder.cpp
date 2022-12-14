/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <rtabmap/core/OccupancyGridBuilder.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_mapping.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>

#include <pcl/io/pcd_io.h>

#include "time_measurer/time_measurer.h"

namespace rtabmap {

OccupancyGridBuilder::OccupancyGridBuilder(const ParametersMap& parameters) :
	cellSize_(Parameters::defaultGridCellSize()),
	sensorBlindRange2d_(Parameters::defaultGridSensorBlindRange2d()),
	localMapBuilder_(),
	coloredOccupancyGrid_(),
	temporaryColoredOccupancyGrid_()
{
	parseParameters(parameters);
}

void OccupancyGridBuilder::parseParameters(const ParametersMap& parameters)
{
	Parameters::parse(parameters, Parameters::kGridCellSize(), cellSize_);
	Parameters::parse(parameters, Parameters::kGridSensorBlindRange2d(), sensorBlindRange2d_);

	sensorBlindRange2dSqr_ = sensorBlindRange2d_ * sensorBlindRange2d_;
	localMapBuilder_.parseParameters(parameters);
	coloredOccupancyGrid_.parseParameters(parameters);
	temporaryColoredOccupancyGrid_.parseParameters(parameters);
}

OccupancyGridBuilder::LocalMap OccupancyGridBuilder::createLocalMap(const Signature & signature) const
{
	LocalMap localMap;
	localMap.LocalMapBuilder::LocalMap::operator=(localMapBuilder_.createLocalMap(signature));
	localMap.sensorBlindRange2dSqr = sensorBlindRange2dSqr_;
	localMap.toSensor = signature.sensorData().laserScan().localTransform();
	return localMap;
}

void OccupancyGridBuilder::addLocalMap(int nodeId, LocalMap localMap)
{
	coloredOccupancyGrid_.addLocalMap(nodeId, std::move(localMap));
}

void OccupancyGridBuilder::addLocalMap(int nodeId, const Transform & pose, LocalMap localMap)
{
	coloredOccupancyGrid_.addLocalMap(nodeId, pose, std::move(localMap));
}

void OccupancyGridBuilder::addTemporaryLocalMap(const Transform& pose, LocalMap localMap)
{
	temporaryColoredOccupancyGrid_.addLocalMap(pose, std::move(localMap));
}

void OccupancyGridBuilder::updatePoses(const std::map<int, Transform>& updatedPoses,
		const std::list<Transform>& updatedTemporaryPoses,
		int lastNodeIdForCachedMap /* -1 */)
{
	coloredOccupancyGrid_.updatePoses(updatedPoses, lastNodeIdForCachedMap);
	temporaryColoredOccupancyGrid_.updatePoses(updatedTemporaryPoses);
}

OccupancyGridBuilder::OccupancyGrid OccupancyGridBuilder::getOccupancyGrid() const
{
	MEASURE_BLOCK_TIME(OccupancyGrid__getOccupancyGrid);
	OccupancyGrid occupancyGrid = coloredOccupancyGrid_.getOccupancyGrid();
	OccupancyGrid temporaryOccupancyGrid = temporaryColoredOccupancyGrid_.getOccupancyGrid();
	
	MapLimits occupancyGridML;
	occupancyGridML.minX = occupancyGrid.minX;
	occupancyGridML.minY = occupancyGrid.minY;
	occupancyGridML.maxX = occupancyGrid.minX + occupancyGrid.grid.cols();
	occupancyGridML.maxY = occupancyGrid.minY + occupancyGrid.grid.rows();

	MapLimits temporaryOccupancyGridML;
	temporaryOccupancyGridML.minX = temporaryOccupancyGrid.minX;
	temporaryOccupancyGridML.minY = temporaryOccupancyGrid.minY;
	temporaryOccupancyGridML.maxX = temporaryOccupancyGrid.minX + temporaryOccupancyGrid.grid.cols();
	temporaryOccupancyGridML.maxY = temporaryOccupancyGrid.minY + temporaryOccupancyGrid.grid.rows();

	MapLimits combinedML = MapLimits::unite(occupancyGridML, temporaryOccupancyGridML);
	OccupancyGrid combined;
	combined.minX = combinedML.minX;
	combined.minY = combinedML.minY;
	combined.grid = OccupancyGrid::GridType::Constant(combinedML.height(), combinedML.width(),
		-1);
	int occupancyGridShiftX = occupancyGridML.minX - combinedML.minX;
	int occupancyGridShiftY = occupancyGridML.minY - combinedML.minY;
	combined.grid.block(occupancyGridShiftY, occupancyGridShiftX,
		occupancyGridML.height(), occupancyGridML.width()) = occupancyGrid.grid;
	int temporaryOccupancyGridShiftX = temporaryOccupancyGridML.minX - combinedML.minX;
	int temporaryOccupancyGridShiftY = temporaryOccupancyGridML.minY - combinedML.minY;
	combined.grid.block(temporaryOccupancyGridShiftY, temporaryOccupancyGridShiftX,
		temporaryOccupancyGridML.height(), temporaryOccupancyGridML.width()) =
		temporaryOccupancyGrid.grid;

	return combined;
}

OccupancyGridBuilder::OccupancyGrid OccupancyGridBuilder::getProbOccupancyGrid() const
{
	MEASURE_BLOCK_TIME(OccupancyGrid__getOccupancyGrid);
	OccupancyGrid occupancyGrid = coloredOccupancyGrid_.getProbOccupancyGrid();
	OccupancyGrid temporaryOccupancyGrid = temporaryColoredOccupancyGrid_.getProbOccupancyGrid();
	
	MapLimits occupancyGridML;
	occupancyGridML.minX = occupancyGrid.minX;
	occupancyGridML.minY = occupancyGrid.minY;
	occupancyGridML.maxX = occupancyGrid.minX + occupancyGrid.grid.cols();
	occupancyGridML.maxY = occupancyGrid.minY + occupancyGrid.grid.rows();

	MapLimits temporaryOccupancyGridML;
	temporaryOccupancyGridML.minX = temporaryOccupancyGrid.minX;
	temporaryOccupancyGridML.minY = temporaryOccupancyGrid.minY;
	temporaryOccupancyGridML.maxX = temporaryOccupancyGrid.minX + temporaryOccupancyGrid.grid.cols();
	temporaryOccupancyGridML.maxY = temporaryOccupancyGrid.minY + temporaryOccupancyGrid.grid.rows();

	MapLimits combinedML = MapLimits::unite(occupancyGridML, temporaryOccupancyGridML);
	OccupancyGrid combined;
	combined.minX = combinedML.minX;
	combined.minY = combinedML.minY;
	combined.grid = OccupancyGrid::GridType::Constant(combinedML.height(), combinedML.width(),
		-1);
	int occupancyGridShiftX = occupancyGridML.minX - combinedML.minX;
	int occupancyGridShiftY = occupancyGridML.minY - combinedML.minY;
	combined.grid.block(occupancyGridShiftY, occupancyGridShiftX,
		occupancyGridML.height(), occupancyGridML.width()) = occupancyGrid.grid;
	int temporaryOccupancyGridShiftX = temporaryOccupancyGridML.minX - combinedML.minX;
	int temporaryOccupancyGridShiftY = temporaryOccupancyGridML.minY - combinedML.minY;
	combined.grid.block(temporaryOccupancyGridShiftY, temporaryOccupancyGridShiftX,
		temporaryOccupancyGridML.height(), temporaryOccupancyGridML.width()) =
		temporaryOccupancyGrid.grid;

	return combined;
}

OccupancyGridBuilder::ColorGrid OccupancyGridBuilder::getColorGrid() const
{
	MEASURE_BLOCK_TIME(OccupancyGrid__getOccupancyGrid);
	ColorGrid colorGrid = coloredOccupancyGrid_.getColorGrid();
	ColorGrid temporaryColorGrid = temporaryColoredOccupancyGrid_.getColorGrid();
	
	MapLimits colorGridML;
	colorGridML.minX = colorGrid.minX;
	colorGridML.minY = colorGrid.minY;
	colorGridML.maxX = colorGrid.minX + colorGrid.grid.cols();
	colorGridML.maxY = colorGrid.minY + colorGrid.grid.rows();

	MapLimits temporaryColorGridML;
	temporaryColorGridML.minX = temporaryColorGrid.minX;
	temporaryColorGridML.minY = temporaryColorGrid.minY;
	temporaryColorGridML.maxX = temporaryColorGrid.minX + temporaryColorGrid.grid.cols();
	temporaryColorGridML.maxY = temporaryColorGrid.minY + temporaryColorGrid.grid.rows();

	MapLimits combinedML = MapLimits::unite(colorGridML, temporaryColorGridML);
	ColorGrid combined;
	combined.minX = combinedML.minX;
	combined.minY = combinedML.minY;
	combined.grid = ColorGrid::GridType::Constant(combinedML.height(), combinedML.width(),
		-1);
	int colorGridShiftX = colorGridML.minX - combinedML.minX;
	int colorGridShiftY = colorGridML.minY - combinedML.minY;
	combined.grid.block(colorGridShiftY, colorGridShiftX,
		colorGridML.height(), colorGridML.width()) = colorGrid.grid;
	int temporaryColorGridShiftX = temporaryColorGridML.minX - combinedML.minX;
	int temporaryColorGridShiftY = temporaryColorGridML.minY - combinedML.minY;
	combined.grid.block(temporaryColorGridShiftY, temporaryColorGridShiftX,
		temporaryColorGridML.height(), temporaryColorGridML.width()) =
		temporaryColorGrid.grid;

	return combined;
}

std::pair<float, float> OccupancyGridBuilder::getGridOrigin() const
{
	MapLimits mapLimits = MapLimits::unite(coloredOccupancyGrid_.mapLimits(),
		temporaryColoredOccupancyGrid_.mapLimits());
	UASSERT(mapLimits.valid());
	float originX = mapLimits.minX * cellSize_;
	float originY = mapLimits.minY * cellSize_;
	return std::make_pair(originX, originY);
}

void OccupancyGridBuilder::resetAll()
{
	coloredOccupancyGrid_ = ColoredOccupancyGrid();
	temporaryColoredOccupancyGrid_ = TemporaryColoredOccupancyGrid();
}

void OccupancyGridBuilder::resetTemporaryMap()
{
	temporaryColoredOccupancyGrid_ = TemporaryColoredOccupancyGrid();
}

}
