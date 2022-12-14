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

OccupancyGridBuilder::OccupancyGridBuilder(const ParametersMap & parameters) :
	cellSize_(Parameters::defaultGridCellSize()),
	occupancyThr_(logodds(Parameters::defaultGridGlobalOccupancyThr())),
	probMiss_(logodds(Parameters::defaultGridGlobalProbMiss())),
	probHit_(logodds(Parameters::defaultGridGlobalProbHit())),
	probClampingMin_(logodds(Parameters::defaultGridGlobalProbClampingMin())),
	probClampingMax_(logodds(Parameters::defaultGridGlobalProbClampingMax())),
	temporaryOccupancyThr_(logodds(Parameters::defaultGridGlobalTemporaryOccupancyThr())),
	temporaryProbMiss_(logodds(Parameters::defaultGridGlobalTemporaryProbMiss())),
	temporaryProbHit_(logodds(Parameters::defaultGridGlobalTemporaryProbHit())),
	temporarilyOccupiedCellColor_(Parameters::defaultGridTemporarilyOccupiedCellColor()),
	showTemporarilyOccupiedCells_(Parameters::defaultGridShowTemporarilyOccupiedCells()),
	maxTemporaryLocalMaps_(Parameters::defaultGridMaxTemporaryLocalMaps()),
	sensorBlindRange2d_(Parameters::defaultGridSensorBlindRange2d()),
	localMapBuilder_()
{
	parseParameters(parameters);
	unknownLogodds_ = probClampingMax_ + 1.0f;
}

void OccupancyGridBuilder::parseParameters(const ParametersMap & parameters)
{
	Parameters::parse(parameters, Parameters::kGridCellSize(), cellSize_);

	if(Parameters::parse(parameters, Parameters::kGridGlobalOccupancyThr(), occupancyThr_))
	{
		occupancyThr_ = logodds(occupancyThr_);
	}
	if(Parameters::parse(parameters, Parameters::kGridGlobalProbMiss(), probMiss_))
	{
		UASSERT_MSG(probMiss_ > 0.0f && probMiss_ <= 0.5f, uFormat("probMiss_=%f", probMiss_).c_str());
		probMiss_ = logodds(probMiss_);
	}
	if(Parameters::parse(parameters, Parameters::kGridGlobalProbHit(), probHit_))
	{
		UASSERT_MSG(probHit_ >= 0.5f && probHit_ < 1.0f, uFormat("probHit_=%f", probHit_).c_str());
		probHit_ = logodds(probHit_);
	}
	if(Parameters::parse(parameters, Parameters::kGridGlobalProbClampingMin(), probClampingMin_))
	{
		probClampingMin_ = logodds(probClampingMin_);
	}
	if(Parameters::parse(parameters, Parameters::kGridGlobalProbClampingMax(), probClampingMax_))
	{
		probClampingMax_ = logodds(probClampingMax_);
	}
	UASSERT(probClampingMax_ > probClampingMin_);

	if(Parameters::parse(parameters, Parameters::kGridGlobalTemporaryOccupancyThr(), temporaryOccupancyThr_))
	{
		temporaryOccupancyThr_ = logodds(temporaryOccupancyThr_);
	}
	if(Parameters::parse(parameters, Parameters::kGridGlobalTemporaryProbMiss(), temporaryProbMiss_))
	{
		UASSERT_MSG(temporaryProbMiss_ > 0.0f && temporaryProbMiss_ <= 0.5f, uFormat("temporaryProbMiss_=%f", temporaryProbMiss_).c_str());
		temporaryProbMiss_ = logodds(temporaryProbMiss_);
	}
	if(Parameters::parse(parameters, Parameters::kGridGlobalTemporaryProbHit(), temporaryProbHit_))
	{
		UASSERT_MSG(temporaryProbHit_ >= 0.5f && temporaryProbHit_ < 1.0f, uFormat("temporaryProbHit_=%f", temporaryProbHit_).c_str());
		temporaryProbHit_ = logodds(temporaryProbHit_);
	}

	Parameters::parse(parameters, Parameters::kGridTemporarilyOccupiedCellColor(), temporarilyOccupiedCellColor_);
	Parameters::parse(parameters, Parameters::kGridShowTemporarilyOccupiedCells(), showTemporarilyOccupiedCells_);
	Parameters::parse(parameters, Parameters::kGridMaxTemporaryLocalMaps(), maxTemporaryLocalMaps_);
	UASSERT(maxTemporaryLocalMaps_ >= 0);

	Parameters::parse(parameters, Parameters::kGridSensorBlindRange2d(), sensorBlindRange2d_);
	sensorBlindRange2dSqr_ = 0.0f;
	if (sensorBlindRange2d_ > 0.0f)
	{
		sensorBlindRange2dSqr_ = sensorBlindRange2d_ * sensorBlindRange2d_;
	}

	localMapBuilder_.parseParameters(parameters);
}

OccupancyGridBuilder::LocalMap OccupancyGridBuilder::createLocalMap(const Signature & signature) const
{
	LocalMapBuilder::LocalMap localMap =
		localMapBuilder_.createLocalMap(signature);
	LocalMap lm;
	lm.numGround = 0;
	lm.numEmpty = localMap.numEmpty;
	lm.numObstacles = localMap.numObstacles;
	lm.points = std::move(localMap.points);
	lm.colors.reserve(lm.colors.size());
	for (const LocalMapBuilder::Color& color : localMap.colors)
	{
		if (color.missing())
		{
			lm.colors.push_back(-1);
		}
		else
		{
			lm.colors.push_back(color.rgb());
		}
	}
	lm.sensorBlindRange2dSqr = sensorBlindRange2dSqr_;
	lm.toSensor = signature.sensorData().laserScan().localTransform();
	return lm;
}

void OccupancyGridBuilder::addLocalMap(int nodeId, LocalMap localMap)
{
	UASSERT(nodeId >= 0);
	UASSERT(map_.nodes.count(nodeId) == 0);
	UASSERT(map_.nodes.empty() || map_.nodes.rbegin()->first < nodeId);
	map_.nodes.emplace(
		std::piecewise_construct,
		std::forward_as_tuple(nodeId),
		std::forward_as_tuple(std::optional<Transform>(), std::move(localMap)));
}

void OccupancyGridBuilder::addLocalMap(int nodeId, const Transform & localPose, LocalMap localMap)
{
	UASSERT(nodeId >= 0);
	UASSERT(map_.nodes.count(nodeId) == 0);
	UASSERT(map_.nodes.empty() || map_.nodes.rbegin()->first < nodeId);
	auto newNodeIt = map_.nodes.emplace(
		std::piecewise_construct,
		std::forward_as_tuple(nodeId),
		std::forward_as_tuple(localPose, std::move(localMap))).first;
	transformLocalMap(newNodeIt->second);
	MapLimits newMapLimits = MapLimits::unite(map_.mapLimits, *(newNodeIt->second.localMapLimits));
	if (map_.mapLimits != newMapLimits)
	{
		createOrResizeMap(map_, newMapLimits);
	}
	deployLocalMap(map_, nodeId);
}

void OccupancyGridBuilder::addTemporaryLocalMap(const Transform & localPose, LocalMap localMap)
{
	int nodeId;
	if (temporaryMap_.nodes.empty())
	{
		nodeId = 0;
	}
	else
	{
		nodeId = temporaryMap_.nodes.rbegin()->first + 1;
	}
	auto newNodeIt = temporaryMap_.nodes.emplace(
		std::piecewise_construct,
		std::forward_as_tuple(nodeId),
		std::forward_as_tuple(localPose, std::move(localMap))).first;
	transformLocalMap(newNodeIt->second);
	MapLimits newMapLimits = MapLimits::unite(temporaryMap_.mapLimits, *(newNodeIt->second.localMapLimits));
	if (temporaryMap_.mapLimits != newMapLimits)
	{
		createOrResizeMap(temporaryMap_, newMapLimits);
	}
	deployLocalMap(temporaryMap_, nodeId);
	if (temporaryMap_.nodes.size() > maxTemporaryLocalMaps_)
	{
		removeLocalMap(temporaryMap_, temporaryMap_.nodes.begin()->first);
	}
}

void OccupancyGridBuilder::cacheCurrentMap()
{
	cachedMap_.reset();
	if(!map_.mapLimits.valid())
	{
		return;
	}

	cachedMap_ = std::make_unique<ColoredOccupancyMap>();
	for (const auto& entry : map_.nodes)
	{
		int nodeId = entry.first;
		const Node& node = entry.second;
		if (!node.localPose.has_value())
		{
			continue;
		}
		cachedMap_->nodes.emplace(
			std::piecewise_construct,
			std::forward_as_tuple(nodeId),
			std::forward_as_tuple(*(node.localPose), LocalMap()));
	}
	cachedMap_->mapLimits = map_.mapLimits;
	cachedMap_->map = map_.map;
	cachedMap_->colors = map_.colors;
	cachedMap_->temporarilyOccupiedCells = map_.temporarilyOccupiedCells;
}

bool OccupancyGridBuilder::checkIfCachedMapCanBeUsed(const std::map<int, Transform> & updatedPoses)
{
	if (cachedMap_ == nullptr)
	{
		return false;
	}
	auto updatedPoseIt = updatedPoses.begin();
	for (const auto& cachedMapIdNode : cachedMap_->nodes)
	{
		if (updatedPoseIt == updatedPoses.end())
		{
			return false;
		}
		if (cachedMapIdNode.first != updatedPoseIt->first ||
			*cachedMapIdNode.second.localPose != updatedPoseIt->second)
		{
			return false;
		}
		++updatedPoseIt;
	}
	return true;
}

void OccupancyGridBuilder::useCachedMap()
{
	clearColoredOccupancyMap(map_);
	auto mapNodeIt = map_.nodes.begin();
	for (const auto& cachedMapIdNode : cachedMap_->nodes)
	{
		UASSERT(mapNodeIt != map_.nodes.end());
		while (mapNodeIt->first != cachedMapIdNode.first)
		{
			++mapNodeIt;
			UASSERT(mapNodeIt != map_.nodes.end());
		}
		mapNodeIt->second.localPose = cachedMapIdNode.second.localPose;
		++mapNodeIt;
	}
	map_.mapLimits = cachedMap_->mapLimits;
	map_.map = cachedMap_->map;
	map_.colors = cachedMap_->colors;
	map_.temporarilyOccupiedCells = cachedMap_->temporarilyOccupiedCells;
}

int OccupancyGridBuilder::tryToUseCachedMap(const std::map<int, Transform> & updatedPoses)
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
	return cachedMap_->nodes.rbegin()->first;
}

void OccupancyGridBuilder::updatePoses(const std::map<int, Transform> & updatedPoses,
		const std::list<Transform> & updatedTemporaryPoses,
		int lastNodeIdForCachedMap /* -1 */)
{
	updatePosesForMap(updatedPoses, lastNodeIdForCachedMap);
	updatePosesForTemporaryMap(updatedTemporaryPoses);
}

void OccupancyGridBuilder::updatePosesForMap(const std::map<int, Transform> & updatedPoses,
		int lastNodeIdForCachedMap /* -1 */)
{
	MEASURE_BLOCK_TIME(OccupancyGrid__updatePosesForMap);
	clearColoredOccupancyMap(map_);
	int lastNodeIdFromCache = tryToUseCachedMap(updatedPoses);

	MapLimits newMapLimits = map_.mapLimits;
	std::list<int> nodeIdsToDeploy;
	auto mapNodeIt = map_.nodes.lower_bound(lastNodeIdFromCache + 1);
	for(auto updatedPoseIt = updatedPoses.lower_bound(lastNodeIdFromCache + 1);
		updatedPoseIt != updatedPoses.end(); ++updatedPoseIt)
	{
		UASSERT(mapNodeIt != map_.nodes.end());
		while (mapNodeIt->first != updatedPoseIt->first)
		{
			++mapNodeIt;
			UASSERT(mapNodeIt != map_.nodes.end());
		}
		int nodeId = mapNodeIt->first;
		Node& mapNode = mapNodeIt->second;
		mapNode.localPose = updatedPoseIt->second;
		transformLocalMap(mapNode);
		newMapLimits = MapLimits::unite(newMapLimits, *(mapNode.localMapLimits));
		nodeIdsToDeploy.push_back(nodeId);

		if (nodeId == lastNodeIdForCachedMap)
		{
			createOrResizeMap(map_, newMapLimits);
			for(auto nodeIdToDeploy : nodeIdsToDeploy)
			{
				deployLocalMap(map_, nodeIdToDeploy);
			}
			cacheCurrentMap();
			nodeIdsToDeploy.clear();
		}

		++mapNodeIt;
	}
	if (newMapLimits.valid() && nodeIdsToDeploy.size())
	{
		createOrResizeMap(map_, newMapLimits);
		for(auto nodeIdToDeploy : nodeIdsToDeploy)
		{
			deployLocalMap(map_, nodeIdToDeploy);
		}
	}
}

void OccupancyGridBuilder::updatePosesForTemporaryMap(const std::list<Transform> & updatedTemporaryPoses)
{
	MEASURE_BLOCK_TIME(OccupancyGrid__updatePosesForTemporaryMap);
	UASSERT(temporaryMap_.nodes.size() == updatedTemporaryPoses.size());
	clearColoredOccupancyMap(temporaryMap_);
	MapLimits newMapLimits = MapLimits();
	auto updatedTemporaryPoseIt = updatedTemporaryPoses.begin();
	for (auto& entry : temporaryMap_.nodes)
	{
		entry.second.localPose = *updatedTemporaryPoseIt;
		transformLocalMap(entry.second);
		newMapLimits = MapLimits::unite(newMapLimits, *(entry.second.localMapLimits));
		++updatedTemporaryPoseIt;
	}
	if (newMapLimits.valid())
	{
		createOrResizeMap(temporaryMap_, newMapLimits);
		for (const auto& entry : temporaryMap_.nodes)
		{
			deployLocalMap(temporaryMap_, entry.first);
		}
	}
}

void OccupancyGridBuilder::transformLocalMap(Node & node)
{
	MEASURE_BLOCK_TIME(OccupancyGrid__transformLocalMap);
	UASSERT(node.localPose.has_value());
	node.transformedLocalPoints2d = Eigen::Matrix2Xi();
	node.transformedLocalPoints2d->resize(2, node.localMap.points.cols());
	Eigen::Matrix3Xf transformedPoints = (node.localPose->toEigen3fRotation() * node.localMap.points).colwise() + node.localPose->toEigen3fTranslation();
	node.localMapLimits = MapLimits();
	for (int i = 0; i < transformedPoints.cols(); i++)
	{
		int x = std::floor(transformedPoints(0, i) / cellSize_);
		int y = std::floor(transformedPoints(1, i) / cellSize_);
		node.transformedLocalPoints2d->coeffRef(0, i) = x;
		node.transformedLocalPoints2d->coeffRef(1, i) = y;
		node.localMapLimits->update(x, y);
	}
}

void OccupancyGridBuilder::createOrResizeMap(ColoredOccupancyMap & map, const MapLimits & newMapLimits)
{
	UASSERT(newMapLimits.valid());
	if(!map.mapLimits.valid())
	{
		MEASURE_BLOCK_TIME(OccupancyGrid__createOrResizeMap__create_map);
		map.mapLimits = newMapLimits;
		map.map = Eigen::MatrixXf::Constant(newMapLimits.height(), newMapLimits.width(), unknownLogodds_);
		map.colors = Eigen::MatrixXi::Constant(newMapLimits.height(), newMapLimits.width(), -1);
	}
	else if(map.mapLimits != newMapLimits)
	{
		MEASURE_BLOCK_TIME(OccupancyGrid__createOrResizeMap__resize_map);
		int dstShiftX = std::max(map.mapLimits.minX - newMapLimits.minX, 0);
		int dstShiftY = std::max(map.mapLimits.minY - newMapLimits.minY, 0);
		int srcShiftX = std::max(newMapLimits.minX - map.mapLimits.minX, 0);
		int srcShiftY = std::max(newMapLimits.minY - map.mapLimits.minY, 0);
		MapLimits intersection = MapLimits::intersect(map.mapLimits, newMapLimits);
		int copyWidth = intersection.width();
		int copyHeight = intersection.height();
		UASSERT(copyWidth > 0 && copyHeight > 0);

		Eigen::MatrixXf newMap = Eigen::MatrixXf::Constant(newMapLimits.height(), newMapLimits.width(), unknownLogodds_);
		Eigen::MatrixXi newColors = Eigen::MatrixXi::Constant(newMapLimits.height(), newMapLimits.width(), -1);

		newMap.block(dstShiftY, dstShiftX, copyHeight, copyWidth) =
			map.map.block(srcShiftY, srcShiftX, copyHeight, copyWidth);
		newColors.block(dstShiftY, dstShiftX, copyHeight, copyWidth) =
			map.colors.block(srcShiftY, srcShiftX, copyHeight, copyWidth);

		map.mapLimits = newMapLimits;
		map.map = std::move(newMap);
		map.colors = std::move(newColors);
	}
}

void OccupancyGridBuilder::createOrResizeMap(TemporaryColoredOccupancyMap & map, const MapLimits & newMapLimits)
{
	UASSERT(newMapLimits.valid());
	if(!map.mapLimits.valid())
	{
		MEASURE_BLOCK_TIME(OccupancyGrid__createOrResizeMap__create_temporary_map);
		map.mapLimits = newMapLimits;
		map.missCounter = Eigen::MatrixXi::Constant(newMapLimits.height(), newMapLimits.width(), 0);
		map.hitCounter = Eigen::MatrixXi::Constant(newMapLimits.height(), newMapLimits.width(), 0);
		map.colors = Eigen::MatrixXi::Constant(newMapLimits.height(), newMapLimits.width(), -1);
	}
	else if(map.mapLimits != newMapLimits)
	{
		MEASURE_BLOCK_TIME(OccupancyGrid__createOrResizeMap__resize_temporary_map);
		int dstShiftX = std::max(map.mapLimits.minX - newMapLimits.minX, 0);
		int dstShiftY = std::max(map.mapLimits.minY - newMapLimits.minY, 0);
		int srcShiftX = std::max(newMapLimits.minX - map.mapLimits.minX, 0);
		int srcShiftY = std::max(newMapLimits.minY - map.mapLimits.minY, 0);
		MapLimits intersection = MapLimits::intersect(map.mapLimits, newMapLimits);
		int copyWidth = intersection.width();
		int copyHeight = intersection.height();
		UASSERT(copyWidth > 0 && copyHeight > 0);

		Eigen::MatrixXi newMissCounter = Eigen::MatrixXi::Constant(newMapLimits.height(), newMapLimits.width(), 0);
		Eigen::MatrixXi newHitCounter = Eigen::MatrixXi::Constant(newMapLimits.height(), newMapLimits.width(), 0);
		Eigen::MatrixXi newColors = Eigen::MatrixXi::Constant(newMapLimits.height(), newMapLimits.width(), -1);

		newMissCounter.block(dstShiftY, dstShiftX, copyHeight, copyWidth) =
			map.missCounter.block(srcShiftY, srcShiftX, copyHeight, copyWidth);
		newHitCounter.block(dstShiftY, dstShiftX, copyHeight, copyWidth) =
			map.hitCounter.block(srcShiftY, srcShiftX, copyHeight, copyWidth);
		newColors.block(dstShiftY, dstShiftX, copyHeight, copyWidth) =
			map.colors.block(srcShiftY, srcShiftX, copyHeight, copyWidth);

		map.mapLimits = newMapLimits;
		map.missCounter = std::move(newMissCounter);
		map.hitCounter = std::move(newHitCounter);
		map.colors = std::move(newColors);
	}
}

void OccupancyGridBuilder::deployLocalMap(ColoredOccupancyMap & map, int nodeId)
{
	MEASURE_BLOCK_TIME(OccupancyGrid__deployLocalMap);
	map.temporarilyOccupiedCells.clear();
	const Node & node = map.nodes.at(nodeId);
	UASSERT(node.transformedLocalPoints2d.has_value());
	for (int i = 0; i < node.transformedLocalPoints2d->cols(); i++)
	{
		int x = node.transformedLocalPoints2d->coeff(0, i) - map.mapLimits.minX;
		int y = node.transformedLocalPoints2d->coeff(1, i) - map.mapLimits.minY;
		UASSERT(x >= 0 && x < map.map.cols() && y >= 0 && y < map.map.rows());

		bool free = (i < node.localMap.numGround + node.localMap.numEmpty);
		if (free)
		{
			float & logodds = map.map(y, x);
			if (node.localMap.sensorBlindRange2dSqr != 0.0f &&
				logodds != unknownLogodds_ && logodds >= occupancyThr_)
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
			if (logodds == unknownLogodds_)
			{
				logodds = 0.0f;
			}
			logodds += probMiss_;
			if (logodds < probClampingMin_)
			{
				logodds = probClampingMin_;
			}
		}
		else
		{
			if (temporarilyOccupiedCellColor_ >= 0)
			{
				const auto & colors = node.localMap.colors;
				int c = colors[i];
				if (c == temporarilyOccupiedCellColor_)
				{
					map.temporarilyOccupiedCells.emplace_back(x, y);
					continue;
				}
			}

			float & logodds = map.map(y, x);
			if (logodds == unknownLogodds_)
			{
				logodds = 0.0f;
			}
			logodds += probHit_;
			if (logodds > probClampingMax_)
			{
				logodds = probClampingMax_;
			}
		}

		int localMapColor = node.localMap.colors[i];
		if (localMapColor != -1)
		{
			int & color = map.colors(y, x);
			color = localMapColor;
		}
	}
}

void OccupancyGridBuilder::deployLocalMap(TemporaryColoredOccupancyMap & map, int nodeId)
{
	MEASURE_BLOCK_TIME(OccupancyGrid__deployLocalMap__temporary);
	const Node & node = map.nodes.at(nodeId);
	UASSERT(node.transformedLocalPoints2d.has_value());
	for (int i = 0; i < node.transformedLocalPoints2d->cols(); i++)
	{
		int x = node.transformedLocalPoints2d->coeff(0, i) - map.mapLimits.minX;
		int y = node.transformedLocalPoints2d->coeff(1, i) - map.mapLimits.minY;
		UASSERT(x >= 0 && x < map.missCounter.cols() && y >= 0 && y < map.missCounter.rows());

		bool free = (i < node.localMap.numGround + node.localMap.numEmpty);
		if (free)
		{
			map.missCounter(y, x) += 1;
		}
		else
		{
			map.hitCounter(y, x) += 1;
		}

		int localMapColor = node.localMap.colors[i];
		int & color = map.colors(y, x);
		color = localMapColor;
	}
}

void OccupancyGridBuilder::removeLocalMap(TemporaryColoredOccupancyMap & map, int nodeId)
{
	MEASURE_BLOCK_TIME(OccupancyGrid__removeLocalMap__temporary);
	const Node & node = map.nodes.at(nodeId);
	UASSERT(node.transformedLocalPoints2d.has_value());
	for (int i = 0; i < node.transformedLocalPoints2d->cols(); i++)
	{
		int x = node.transformedLocalPoints2d->coeff(0, i) - map.mapLimits.minX;
		int y = node.transformedLocalPoints2d->coeff(1, i) - map.mapLimits.minY;
		UASSERT(x >= 0 && x < map.missCounter.cols() && y >= 0 && y < map.missCounter.rows());

		bool free = (i < node.localMap.numGround + node.localMap.numEmpty);
		if (free)
		{
			int & misses = map.missCounter(y, x);
			misses -= 1;
			UASSERT(misses >= 0);
		}
		else
		{
			int & hits = map.hitCounter(y, x);
			hits -= 1;
			UASSERT(hits >= 0);
		}
	}
	map.nodes.erase(nodeId);

	MapLimits newMapLimits;
	for (const auto& entry : map.nodes)
	{
		newMapLimits = MapLimits::unite(newMapLimits, *(entry.second.localMapLimits));
	}
	createOrResizeMap(map, newMapLimits);
}

void OccupancyGridBuilder::clearColoredOccupancyMap(ColoredOccupancyMap& map)
{
	for (auto& entry : map.nodes)
	{
		entry.second.localPose.reset();
		entry.second.transformedLocalPoints2d.reset();
		entry.second.localMapLimits.reset();
	}
	map.mapLimits = MapLimits();
	map.map = Eigen::MatrixXf();
	map.colors = Eigen::MatrixXi();
	map.temporarilyOccupiedCells.clear();
}

void OccupancyGridBuilder::clearColoredOccupancyMap(TemporaryColoredOccupancyMap& map)
{
	for (auto& entry : map.nodes)
	{
		entry.second.localPose.reset();
		entry.second.transformedLocalPoints2d.reset();
		entry.second.localMapLimits.reset();
	}
	map.mapLimits = MapLimits();
	map.missCounter = Eigen::MatrixXi();
	map.hitCounter = Eigen::MatrixXi();
	map.colors = Eigen::MatrixXi();
}

OccupancyGridBuilder::OccupancyGrid OccupancyGridBuilder::getOccupancyGrid(float & minX, float & minY) const
{
	MEASURE_BLOCK_TIME(OccupancyGrid__getOccupancyGrid);
	MapLimits occupancyMapLimits = MapLimits::unite(map_.mapLimits, temporaryMap_.mapLimits);
	UASSERT(occupancyMapLimits.valid());
	minX = occupancyMapLimits.minX * cellSize_;
	minY = occupancyMapLimits.minY * cellSize_;
	OccupancyGrid occupancyGrid =
		OccupancyGrid::Constant(occupancyMapLimits.height(), occupancyMapLimits.width(), -1);

	if (map_.mapLimits.valid())
	{
		int shiftX = map_.mapLimits.minX - occupancyMapLimits.minX;
		int shiftY = map_.mapLimits.minY - occupancyMapLimits.minY;
		for(int x = 0; x < map_.map.cols(); ++x)
		{
			for(int y = 0; y < map_.map.rows(); ++y)
			{
				float logodds = map_.map(y, x);
				if(logodds == unknownLogodds_)
				{
					occupancyGrid(y + shiftY, x + shiftX) = -1;
				}
				else if(logodds >= occupancyThr_)
				{
					occupancyGrid(y + shiftY, x + shiftX) = 100;
				}
				else
				{
					occupancyGrid(y + shiftY, x + shiftX) = 0;
				}
			}
		}
		if (showTemporarilyOccupiedCells_)
		{
			for (const auto& pair : map_.temporarilyOccupiedCells)
			{
				int x = pair.first;
				int y = pair.second;
				occupancyGrid(y + shiftY, x + shiftX) = 100;
			}
		}
	}

	if (temporaryMap_.mapLimits.valid())
	{
		int shiftX = temporaryMap_.mapLimits.minX - occupancyMapLimits.minX;
		int shiftY = temporaryMap_.mapLimits.minY - occupancyMapLimits.minY;
		for(int x = 0; x < temporaryMap_.missCounter.cols(); ++x)
		{
			for(int y = 0; y < temporaryMap_.missCounter.rows(); ++y)
			{
				int misses = temporaryMap_.missCounter(y, x);
				int hits = temporaryMap_.hitCounter(y, x);
				float logodds = misses * temporaryProbMiss_ + hits * temporaryProbHit_;
				if(misses == 0 && hits == 0)
				{
					continue;
				}
				else if(logodds >= temporaryOccupancyThr_)
				{
					occupancyGrid(y + shiftY, x + shiftX) = 100;
				}
				else
				{
					occupancyGrid(y + shiftY, x + shiftX) = 0;
				}
			}
		}
	}

	return occupancyGrid;
}

OccupancyGridBuilder::OccupancyGrid OccupancyGridBuilder::getProbOccupancyGrid(float & minX, float & minY) const
{
	MEASURE_BLOCK_TIME(OccupancyGrid__getProbOccupancyGrid);
	MapLimits occupancyMapLimits = MapLimits::unite(map_.mapLimits, temporaryMap_.mapLimits);
	UASSERT(occupancyMapLimits.valid());
	minX = occupancyMapLimits.minX * cellSize_;
	minY = occupancyMapLimits.minY * cellSize_;
	OccupancyGrid occupancyGrid =
		OccupancyGrid::Constant(occupancyMapLimits.height(), occupancyMapLimits.width(), -1);

	if (map_.mapLimits.valid())
	{
		int shiftX = map_.mapLimits.minX - occupancyMapLimits.minX;
		int shiftY = map_.mapLimits.minY - occupancyMapLimits.minY;
		for(int x = 0; x < map_.map.cols(); ++x)
		{
			for(int y = 0; y < map_.map.rows(); ++y)
			{
				float logodds = map_.map(y, x);
				if(logodds == unknownLogodds_)
				{
					occupancyGrid(y + shiftY, x + shiftX) = -1;
				}
				else
				{
					occupancyGrid(y + shiftY, x + shiftX) = probability(logodds) * 100;
				}
			}
		}
		if (showTemporarilyOccupiedCells_)
		{
			for (const auto& pair : map_.temporarilyOccupiedCells)
			{
				int x = pair.first;
				int y = pair.second;
				occupancyGrid(y + shiftY, x + shiftX) = 100;
			}
		}
	}

	if (temporaryMap_.mapLimits.valid())
	{
		int shiftX = temporaryMap_.mapLimits.minX - occupancyMapLimits.minX;
		int shiftY = temporaryMap_.mapLimits.minY - occupancyMapLimits.minY;
		for(int x = 0; x < temporaryMap_.missCounter.cols(); ++x)
		{
			for(int y = 0; y < temporaryMap_.missCounter.rows(); ++y)
			{
				int misses = temporaryMap_.missCounter(y, x);
				int hits = temporaryMap_.hitCounter(y, x);
				float logodds = misses * temporaryProbMiss_ + hits * temporaryProbHit_;
				if(misses == 0 && hits == 0)
				{
					continue;
				}
				else
				{
					occupancyGrid(y + shiftY, x + shiftX) = logodds;
				}
			}
		}
	}
	return occupancyGrid;
}

OccupancyGridBuilder::ColorGrid OccupancyGridBuilder::getColorGrid(float & minX, float & minY) const
{
	MEASURE_BLOCK_TIME(OccupancyGrid__getColorGrid);
	MapLimits colorsMapLimits = MapLimits::unite(map_.mapLimits, temporaryMap_.mapLimits);
	UASSERT(colorsMapLimits.valid());
	minX = colorsMapLimits.minX * cellSize_;
	minY = colorsMapLimits.minY * cellSize_;
	ColorGrid colorGrid =
		ColorGrid::Constant(colorsMapLimits.height(), colorsMapLimits.width(), -1);

	if (map_.mapLimits.valid())
	{
		int shiftX = std::max(map_.mapLimits.minX - colorsMapLimits.minX, 0);
		int shiftY = std::max(map_.mapLimits.minY - colorsMapLimits.minY, 0);
		colorGrid.block(shiftY, shiftX, map_.mapLimits.height(), map_.mapLimits.width()) =
			map_.colors;

		if (showTemporarilyOccupiedCells_)
		{
			for (const auto& pair : map_.temporarilyOccupiedCells)
			{
				int x = pair.first;
				int y = pair.second;
				colorGrid(y + shiftY, x + shiftX) = temporarilyOccupiedCellColor_;
			}
		}
	}

	if (temporaryMap_.mapLimits.valid())
	{
		int shiftX = std::max(temporaryMap_.mapLimits.minX - colorsMapLimits.minX, 0);
		int shiftY = std::max(temporaryMap_.mapLimits.minY - colorsMapLimits.minY, 0);
		colorGrid.block(shiftY, shiftX, temporaryMap_.mapLimits.height(), temporaryMap_.mapLimits.width()) =
			temporaryMap_.colors;
	}

	return colorGrid;
}

float OccupancyGridBuilder::cellSize() const
{
	return cellSize_;
}

std::pair<float, float> OccupancyGridBuilder::getGridOrigin() const
{
	MapLimits mapLimits = MapLimits::unite(map_.mapLimits, temporaryMap_.mapLimits);
	UASSERT(mapLimits.valid());
	float originX = mapLimits.minX * cellSize_;
	float originY = mapLimits.minY * cellSize_;
	return std::make_pair(originX, originY);
}

int OccupancyGridBuilder::maxTemporaryLocalMaps() const
{
	return maxTemporaryLocalMaps_;
}

const std::map<int, OccupancyGridBuilder::Node> & OccupancyGridBuilder::nodes() const
{
	return map_.nodes;
}

const cv::Mat & OccupancyGridBuilder::lastDilatedSemantic() const
{
	return lastDilatedSemantic_;
}

void OccupancyGridBuilder::resetAll()
{
	map_ = ColoredOccupancyMap();
	temporaryMap_ = TemporaryColoredOccupancyMap();
	cachedMap_.reset();
	lastDilatedSemantic_ = cv::Mat();
}

void OccupancyGridBuilder::resetTemporaryMap()
{
	temporaryMap_ = TemporaryColoredOccupancyMap();
}

}
