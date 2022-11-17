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
	parameters_(parameters),
	cloudDecimation_(Parameters::defaultGridDepthDecimation()),
	cloudMaxDepth_(Parameters::defaultGridRangeMax()),
	cloudMinDepth_(Parameters::defaultGridRangeMin()),
	footprintLength_(Parameters::defaultGridFootprintLength()),
	footprintWidth_(Parameters::defaultGridFootprintWidth()),
	footprintHeight_(Parameters::defaultGridFootprintHeight()),
	scanDecimation_(Parameters::defaultGridScanDecimation()),
	cellSize_(Parameters::defaultGridCellSize()),
	preVoxelFiltering_(Parameters::defaultGridPreVoxelFiltering()),
	occupancyFromDepth_(Parameters::defaultGridFromDepth()),
	projMapFrame_(Parameters::defaultGridMapFrameProjection()),
	maxObstacleHeight_(Parameters::defaultGridMaxObstacleHeight()),
	normalKSearch_(Parameters::defaultGridNormalK()),
	maxGroundAngle_(Parameters::defaultGridMaxGroundAngle()*M_PI/180.0f),
	clusterRadius_(Parameters::defaultGridClusterRadius()),
	minClusterSize_(Parameters::defaultGridMinClusterSize()),
	flatObstaclesDetected_(Parameters::defaultGridFlatObstacleDetected()),
	minGroundHeight_(Parameters::defaultGridMinGroundHeight()),
	maxGroundHeight_(Parameters::defaultGridMaxGroundHeight()),
	normalsSegmentation_(Parameters::defaultGridNormalsSegmentation()),
	noiseFilteringRadius_(Parameters::defaultGridNoiseFilteringRadius()),
	noiseFilteringMinNeighbors_(Parameters::defaultGridNoiseFilteringMinNeighbors()),
	scan2dUnknownSpaceFilled_(Parameters::defaultGridScan2dUnknownSpaceFilled()),
	rayTracing_(Parameters::defaultGridRayTracing()),
	footprintRadius_(Parameters::defaultGridGlobalFootprintRadius()),
	occupancyThr_(Parameters::defaultGridGlobalOccupancyThr()),
	probMiss_(logodds(Parameters::defaultGridGlobalProbMiss())),
	probHit_(logodds(Parameters::defaultGridGlobalProbHit())),
	probClampingMin_(logodds(Parameters::defaultGridGlobalProbClampingMin())),
	probClampingMax_(logodds(Parameters::defaultGridGlobalProbClampingMax())),
	temporaryOccupancyThr_(Parameters::defaultGridGlobalTemporaryOccupancyThr()),
	temporaryProbMiss_(logodds(Parameters::defaultGridGlobalTemporaryProbMiss())),
	temporaryProbHit_(logodds(Parameters::defaultGridGlobalTemporaryProbHit())),
	semanticDilation_(Parameters::defaultGridSemanticDilation()),
	minSemanticRange_(Parameters::defaultGridMinSemanticRange()),
	maxSemanticRange_(Parameters::defaultGridMaxSemanticRange()),
	temporarilyOccupiedCellColor_(Parameters::defaultGridTemporarilyOccupiedCellColor()),
	showTemporarilyOccupiedCells_(Parameters::defaultGridShowTemporarilyOccupiedCells()),
	maxTemporaryLocalMaps_(Parameters::defaultGridMaxTemporaryLocalMaps())
{
	parseParameters(parameters);
}

void OccupancyGridBuilder::parseParameters(const ParametersMap & parameters)
{
	Parameters::parse(parameters, Parameters::kGridFromDepth(), occupancyFromDepth_);
	Parameters::parse(parameters, Parameters::kGridDepthDecimation(), cloudDecimation_);
	if(cloudDecimation_ == 0)
	{
		cloudDecimation_ = 1;
	}
	Parameters::parse(parameters, Parameters::kGridRangeMin(), cloudMinDepth_);
	Parameters::parse(parameters, Parameters::kGridRangeMax(), cloudMaxDepth_);
	Parameters::parse(parameters, Parameters::kGridFootprintLength(), footprintLength_);
	Parameters::parse(parameters, Parameters::kGridFootprintWidth(), footprintWidth_);
	Parameters::parse(parameters, Parameters::kGridFootprintHeight(), footprintHeight_);
	Parameters::parse(parameters, Parameters::kGridScanDecimation(), scanDecimation_);
	Parameters::parse(parameters, Parameters::kGridCellSize(), cellSize_);

	Parameters::parse(parameters, Parameters::kGridPreVoxelFiltering(), preVoxelFiltering_);
	Parameters::parse(parameters, Parameters::kGridMapFrameProjection(), projMapFrame_);
	Parameters::parse(parameters, Parameters::kGridMaxObstacleHeight(), maxObstacleHeight_);
	Parameters::parse(parameters, Parameters::kGridMinGroundHeight(), minGroundHeight_);
	Parameters::parse(parameters, Parameters::kGridMaxGroundHeight(), maxGroundHeight_);
	Parameters::parse(parameters, Parameters::kGridNormalK(), normalKSearch_);
	if(Parameters::parse(parameters, Parameters::kGridMaxGroundAngle(), maxGroundAngle_))
	{
		maxGroundAngle_ *= M_PI/180.0f;
	}
	Parameters::parse(parameters, Parameters::kGridClusterRadius(), clusterRadius_);
	UASSERT_MSG(clusterRadius_ > 0.0f, uFormat("Param name is \"%s\"", Parameters::kGridClusterRadius().c_str()).c_str());
	Parameters::parse(parameters, Parameters::kGridMinClusterSize(), minClusterSize_);
	Parameters::parse(parameters, Parameters::kGridFlatObstacleDetected(), flatObstaclesDetected_);
	Parameters::parse(parameters, Parameters::kGridNormalsSegmentation(), normalsSegmentation_);
	Parameters::parse(parameters, Parameters::kGridNoiseFilteringRadius(), noiseFilteringRadius_);
	Parameters::parse(parameters, Parameters::kGridNoiseFilteringMinNeighbors(), noiseFilteringMinNeighbors_);
	Parameters::parse(parameters, Parameters::kGridScan2dUnknownSpaceFilled(), scan2dUnknownSpaceFilled_);
	Parameters::parse(parameters, Parameters::kGridRayTracing(), rayTracing_);
	Parameters::parse(parameters, Parameters::kGridGlobalFootprintRadius(), footprintRadius_);

	Parameters::parse(parameters, Parameters::kGridGlobalOccupancyThr(), occupancyThr_);
	if(Parameters::parse(parameters, Parameters::kGridGlobalProbMiss(), probMiss_))
	{
		UASSERT_MSG(probMiss_ > 0.0f && probMiss_ < 0.5f, uFormat("probMiss_=%f", probMiss_).c_str());
		probMiss_ = logodds(probMiss_);
	}
	if(Parameters::parse(parameters, Parameters::kGridGlobalProbHit(), probHit_))
	{
		UASSERT_MSG(probHit_ > 0.5f && probHit_ < 1.0f, uFormat("probHit_=%f", probHit_).c_str());
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

	Parameters::parse(parameters, Parameters::kGridGlobalTemporaryOccupancyThr(), temporaryOccupancyThr_);
	if(Parameters::parse(parameters, Parameters::kGridGlobalTemporaryProbMiss(), temporaryProbMiss_))
	{
		UASSERT_MSG(temporaryProbMiss_ > 0.0f && temporaryProbMiss_ < 0.5f, uFormat("temporaryProbMiss_=%f", temporaryProbMiss_).c_str());
		temporaryProbMiss_ = logodds(temporaryProbMiss_);
	}
	if(Parameters::parse(parameters, Parameters::kGridGlobalTemporaryProbHit(), temporaryProbHit_))
	{
		UASSERT_MSG(temporaryProbHit_ > 0.5f && temporaryProbHit_ < 1.0f, uFormat("temporaryProbHit_=%f", temporaryProbHit_).c_str());
		temporaryProbHit_ = logodds(temporaryProbHit_);
	}

	Parameters::parse(parameters, Parameters::kGridSemanticDilation(), semanticDilation_);
	UASSERT(semanticDilation_ >= 0);

	Parameters::parse(parameters, Parameters::kGridMinSemanticRange(), minSemanticRange_);
	minSemanticRangeSqr_ = 0.0f;
	if(minSemanticRange_ > 0.0f)
	{
		minSemanticRangeSqr_ = minSemanticRange_ * minSemanticRange_;
	}
	Parameters::parse(parameters, Parameters::kGridMaxSemanticRange(), maxSemanticRange_);
	maxSemanticRangeSqr_ = 0.0f;
	if(maxSemanticRange_ > 0.0f)
	{
		maxSemanticRangeSqr_ = maxSemanticRange_ * maxSemanticRange_;
	}

	Parameters::parse(parameters, Parameters::kGridTemporarilyOccupiedCellColor(), temporarilyOccupiedCellColor_);
	Parameters::parse(parameters, Parameters::kGridShowTemporarilyOccupiedCells(), showTemporarilyOccupiedCells_);
	Parameters::parse(parameters, Parameters::kGridMaxTemporaryLocalMaps(), maxTemporaryLocalMaps_);
	UASSERT(maxTemporaryLocalMaps_ >= 0);

	// convert ROI from string to vector
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kGridDepthRoiRatios())) != parameters.end())
	{
		std::list<std::string> strValues = uSplit(iter->second, ' ');
		if(strValues.size() != 4)
		{
			ULOGGER_ERROR("The number of values must be 4 (%s=\"%s\")", iter->first.c_str(), iter->second.c_str());
		}
		else
		{
			std::vector<float> tmpValues(4);
			unsigned int i=0;
			for(std::list<std::string>::iterator jter = strValues.begin(); jter!=strValues.end(); ++jter)
			{
				tmpValues[i] = uStr2Float(*jter);
				++i;
			}

			if(tmpValues[0] >= 0 && tmpValues[0] < 1 && tmpValues[0] < 1.0f-tmpValues[1] &&
				tmpValues[1] >= 0 && tmpValues[1] < 1 && tmpValues[1] < 1.0f-tmpValues[0] &&
				tmpValues[2] >= 0 && tmpValues[2] < 1 && tmpValues[2] < 1.0f-tmpValues[3] &&
				tmpValues[3] >= 0 && tmpValues[3] < 1 && tmpValues[3] < 1.0f-tmpValues[2])
			{
				roiRatios_ = tmpValues;
			}
			else
			{
				ULOGGER_ERROR("The roi ratios are not valid (%s=\"%s\")", iter->first.c_str(), iter->second.c_str());
			}
		}
	}

	if(maxGroundHeight_ == 0.0f && !normalsSegmentation_)
	{
		UWARN("\"%s\" should be not equal to 0 if not using normals "
				"segmentation approach. Setting it to cell size (%f).",
				Parameters::kGridMaxGroundHeight().c_str(), cellSize_);
		maxGroundHeight_ = cellSize_;
	}
	if(maxGroundHeight_ != 0.0f &&
		maxObstacleHeight_ != 0.0f &&
		maxObstacleHeight_ < maxGroundHeight_)
	{
		UWARN("\"%s\" should be lower than \"%s\", setting \"%s\" to 0 (disabled).",
				Parameters::kGridMaxGroundHeight().c_str(),
				Parameters::kGridMaxObstacleHeight().c_str(),
				Parameters::kGridMaxObstacleHeight().c_str());
		maxObstacleHeight_ = 0;
	}
	if(maxGroundHeight_ != 0.0f &&
		minGroundHeight_ != 0.0f &&
		maxGroundHeight_ < minGroundHeight_)
	{
		UWARN("\"%s\" should be lower than \"%s\", setting \"%s\" to 0 (disabled).",
				Parameters::kGridMinGroundHeight().c_str(),
				Parameters::kGridMaxGroundHeight().c_str(),
				Parameters::kGridMinGroundHeight().c_str());
		minGroundHeight_ = 0;
	}
}

OccupancyGridBuilder::LocalMap OccupancyGridBuilder::createLocalMap(const Signature & signature) const
{
	MEASURE_BLOCK_TIME(OccupancyGrid__createLocalMap);
	cv::Mat groundCells;
	cv::Mat emptyCells;
	cv::Mat obstacleCells;
	cv::Point3f viewPoint;

	if(signature.sensorData().laserScan().is2d() && !occupancyFromDepth_)
	{
		UDEBUG("2D laser scan");
		// 2D
		viewPoint = cv::Point3f(
				signature.sensorData().laserScan().localTransform().x(),
				signature.sensorData().laserScan().localTransform().y(),
				signature.sensorData().laserScan().localTransform().z());

		LaserScan scan = signature.sensorData().laserScan();
		if(cloudMinDepth_ > 0.0f)
		{
			scan = util3d::rangeFiltering(scan, cloudMinDepth_, 0.0f);
		}

		float maxRange = cloudMaxDepth_;
		if(cloudMaxDepth_>0.0f && signature.sensorData().laserScan().rangeMax()>0.0f)
		{
			maxRange = cloudMaxDepth_ < signature.sensorData().laserScan().rangeMax()?cloudMaxDepth_:signature.sensorData().laserScan().rangeMax();
		}
		else if(scan2dUnknownSpaceFilled_ && signature.sensorData().laserScan().rangeMax()>0.0f)
		{
			maxRange = signature.sensorData().laserScan().rangeMax();
		}
		util3d::occupancy2DFromLaserScan(
				util3d::transformLaserScan(scan, signature.sensorData().laserScan().localTransform()).data(),
				cv::Mat(),
				viewPoint,
				emptyCells,
				obstacleCells,
				cellSize_,
				scan2dUnknownSpaceFilled_,
				maxRange);
	}
	else
	{
		if(!occupancyFromDepth_)
		{
			if(!signature.sensorData().laserScan().isEmpty())
			{
				UDEBUG("3D laser scan");
				const Transform & t = signature.sensorData().laserScan().localTransform();
				MEASURE_TIME_FROM_HERE(OccupancyGrid__downsample);
				LaserScan scan = util3d::downsample(signature.sensorData().laserScan(), scanDecimation_);
				STOP_TIME_MEASUREMENT(OccupancyGrid__downsample);
				if(cloudMinDepth_ > 0.0f || cloudMaxDepth_ > 0.0f)
				{
					MEASURE_BLOCK_TIME(OccupancyGrid__rangeFiltering);
					scan = util3d::rangeFiltering(scan, cloudMinDepth_, 0);
				}

				if (signature.sensorData().images().size())
				{
					std::vector<cv::Mat> semantics;
					semantics.reserve(signature.sensorData().images().size());
					for (int i = 0; i < signature.sensorData().images().size(); i++)
					{
						cv::Mat semantic = signature.sensorData().images()[i];
						if (semanticDilation_ > 0)
						{
							semantic = dilate(semantic);
						}
						semantics.push_back(semantic);
					}
					scan = addSemanticToLaserScan(scan, semantics, signature.sensorData().cameraModels());
				}

				// update viewpoint
				viewPoint = cv::Point3f(t.x(), t.y(), t.z());

				UDEBUG("scan format=%d", scan.format());
				createLocalMap(scan, signature.getPose(), groundCells, emptyCells, obstacleCells, viewPoint);
			}
			else
			{
				UWARN("Cannot create local map, scan is empty (signature=%d, %s=false).", signature.id(), Parameters::kGridFromDepth().c_str());
			}
		}
		else
		{
			pcl::IndicesPtr indices(new std::vector<int>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
			UDEBUG("Depth image : decimation=%d max=%f min=%f",
					cloudDecimation_,
					cloudMaxDepth_,
					cloudMinDepth_);
			cloud = util3d::cloudRGBFromSensorData(
					signature.sensorData(),
					cloudDecimation_,
					cloudMaxDepth_,
					cloudMinDepth_,
					indices.get(),
					parameters_,
					roiRatios_);

			// update viewpoint
			if(signature.sensorData().cameraModels().size())
			{
				// average of all local transforms
				float sum = 0;
				for(unsigned int i=0; i<signature.sensorData().cameraModels().size(); ++i)
				{
					const Transform & t = signature.sensorData().cameraModels()[i].localTransform();
					if(!t.isNull())
					{
						viewPoint.x += t.x();
						viewPoint.y += t.y();
						viewPoint.z += t.z();
						sum += 1.0f;
					}
				}
				if(sum > 0.0f)
				{
					viewPoint.x /= sum;
					viewPoint.y /= sum;
					viewPoint.z /= sum;
				}
			}
			else
			{
				const Transform & t = signature.sensorData().stereoCameraModel().localTransform();
				viewPoint = cv::Point3f(t.x(), t.y(), t.z());
			}
			createLocalMap(LaserScan(util3d::laserScanFromPointCloud(*cloud, indices), 0, 0.0f), signature.getPose(), groundCells, emptyCells, obstacleCells, viewPoint);
		}
	}

	return cvMatsToLocalMap(groundCells, emptyCells, obstacleCells);
}

void OccupancyGridBuilder::createLocalMap(
		const LaserScan & scan,
		const Transform & pose,
		cv::Mat & groundCells,
		cv::Mat & emptyCells,
		cv::Mat & obstacleCells,
		cv::Point3f & viewPoint) const
{
	if(projMapFrame_)
	{
		// we should rotate viewPoint in /map frame
		float roll, pitch, yaw;
		pose.getEulerAngles(roll, pitch, yaw);
		Transform viewpointRotated = Transform(0,0,0,roll,pitch,0) * Transform(viewPoint.x, viewPoint.y, viewPoint.z, 0,0,0);
		viewPoint.x = viewpointRotated.x();
		viewPoint.y = viewpointRotated.y();
		viewPoint.z = viewpointRotated.z();
	}

	if(scan.size())
	{
		pcl::IndicesPtr groundIndices(new std::vector<int>);
		pcl::IndicesPtr obstaclesIndices(new std::vector<int>);
		cv::Mat groundCloud;
		cv::Mat obstaclesCloud;

		if(scan.hasRGB() && scan.hasNormals())
		{
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud = util3d::laserScanToPointCloudRGBNormal(scan, scan.localTransform());
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudSegmented = segmentCloud<pcl::PointXYZRGBNormal>(cloud, pcl::IndicesPtr(new std::vector<int>), pose, viewPoint, groundIndices, obstaclesIndices);
			UDEBUG("groundIndices=%d, obstaclesIndices=%d", (int)groundIndices->size(), (int)obstaclesIndices->size());
			util3d::occupancy2DFromGroundObstacles<pcl::PointXYZRGBNormal>(cloudSegmented, groundIndices, obstaclesIndices, groundCells, obstacleCells, cellSize_);
		}
		else if(scan.hasRGB())
		{
			MEASURE_TIME_FROM_HERE(OccupancyGrid__scan2PCL);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::laserScanToPointCloudRGB(scan, scan.localTransform());
			STOP_TIME_MEASUREMENT(OccupancyGrid__scan2PCL);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSegmented = segmentCloud<pcl::PointXYZRGB>(cloud, pcl::IndicesPtr(new std::vector<int>), pose, viewPoint, groundIndices, obstaclesIndices);
			UDEBUG("groundIndices=%d, obstaclesIndices=%d", (int)groundIndices->size(), (int)obstaclesIndices->size());
			MEASURE_TIME_FROM_HERE(OccupancyGrid__PCL2cvMats);
			util3d::occupancy2DFromGroundObstacles<pcl::PointXYZRGB>(cloudSegmented, groundIndices, obstaclesIndices, groundCells, obstacleCells, cellSize_);
			STOP_TIME_MEASUREMENT(OccupancyGrid__PCL2cvMats);
		}
		else if(scan.hasNormals())
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud = util3d::laserScanToPointCloudNormal(scan, scan.localTransform());
			pcl::PointCloud<pcl::PointNormal>::Ptr cloudSegmented = segmentCloud<pcl::PointNormal>(cloud, pcl::IndicesPtr(new std::vector<int>), pose, viewPoint, groundIndices, obstaclesIndices);
			UDEBUG("groundIndices=%d, obstaclesIndices=%d", (int)groundIndices->size(), (int)obstaclesIndices->size());
			util3d::occupancy2DFromGroundObstacles<pcl::PointNormal>(cloudSegmented, groundIndices, obstaclesIndices, groundCells, obstacleCells, cellSize_);
		}
		else
		{
			MEASURE_TIME_FROM_HERE(OccupancyGrid__scan2PCL);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::laserScanToPointCloud(scan, scan.localTransform());
			STOP_TIME_MEASUREMENT(OccupancyGrid__scan2PCL);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmented = segmentCloud<pcl::PointXYZ>(cloud, pcl::IndicesPtr(new std::vector<int>), pose, viewPoint, groundIndices, obstaclesIndices);
			UDEBUG("groundIndices=%d, obstaclesIndices=%d", (int)groundIndices->size(), (int)obstaclesIndices->size());
			MEASURE_TIME_FROM_HERE(OccupancyGrid__PCL2cvMats);
			util3d::occupancy2DFromGroundObstacles<pcl::PointXYZ>(cloudSegmented, groundIndices, obstaclesIndices, groundCells, obstacleCells, cellSize_);
			STOP_TIME_MEASUREMENT(OccupancyGrid__PCL2cvMats);
		}

		if(rayTracing_ && (!obstacleCells.empty() || !groundCells.empty()))
		{
			MEASURE_BLOCK_TIME(OccupancyGrid__rayTracing);
			cv::Mat laserScanNoHit = groundCells;
			cv::Mat laserScan = obstacleCells;
			groundCells = cv::Mat();
			obstacleCells = cv::Mat();
			util3d::occupancy2DFromLaserScan(
					laserScan,
					laserScanNoHit,
					viewPoint,
					emptyCells,
					obstacleCells,
					cellSize_,
					false, // don't fill unknown space
					cloudMaxDepth_);
		}
	}
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr OccupancyGridBuilder::segmentCloud(
		const typename pcl::PointCloud<PointT>::Ptr & cloudIn,
		const pcl::IndicesPtr & indicesIn,
		const Transform & pose,
		const cv::Point3f & viewPoint,
		pcl::IndicesPtr & groundIndices,
		pcl::IndicesPtr & obstaclesIndices,
		pcl::IndicesPtr * flatObstacles /* nullptr */) const
{
	MEASURE_BLOCK_TIME(OccupancyGrid__segmentCloud);
	groundIndices.reset(new std::vector<int>);
	obstaclesIndices.reset(new std::vector<int>);
	if(flatObstacles)
	{
		flatObstacles->reset(new std::vector<int>);
	}

	typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::IndicesPtr indices(new std::vector<int>);

	if(preVoxelFiltering_)
	{
		// voxelize to grid cell size
		cloud = util3d::voxelize(cloudIn, indicesIn, cellSize_);

		indices->resize(cloud->size());
		for(unsigned int i=0; i<indices->size(); ++i)
		{
			indices->at(i) = i;
		}
	}
	else
	{
		cloud = cloudIn;
		if(indicesIn->empty() && cloud->is_dense)
		{
			indices->resize(cloud->size());
			for(unsigned int i=0; i<indices->size(); ++i)
			{
				indices->at(i) = i;
			}
		}
		else
		{
			indices = indicesIn;
		}
	}

	// add pose rotation without yaw
	float roll, pitch, yaw;
	pose.getEulerAngles(roll, pitch, yaw);
	// cloud = util3d::transformPointCloud(cloud, Transform(0,0, projMapFrame_?pose.z():0, roll, pitch, 0));

	// filter footprint
	if(footprintLength_ > 0.0f || footprintWidth_ > 0.0f || footprintHeight_ > 0.0f)
	{
		indices = util3d::cropBox(
				cloud,
				indices,
				Eigen::Vector4f(
						footprintLength_>0.0f?-footprintLength_/2.0f:std::numeric_limits<int>::min(),
						footprintWidth_>0.0f&&footprintLength_>0.0f?-footprintWidth_/2.0f:std::numeric_limits<int>::min(),
						0,
						1),
				Eigen::Vector4f(
						footprintLength_>0.0f?footprintLength_/2.0f:std::numeric_limits<int>::max(),
						footprintWidth_>0.0f&&footprintLength_>0.0f?footprintWidth_/2.0f:std::numeric_limits<int>::max(),
						footprintHeight_>0.0f&&footprintLength_>0.0f&&footprintWidth_>0.0f?footprintHeight_:std::numeric_limits<int>::max(),
						1),
				Transform::getIdentity(),
				true);
	}

	// filter ground/obstacles zone
	if(minGroundHeight_ != 0.0f || maxObstacleHeight_ != 0.0f)
	{
		indices = util3d::passThrough(cloud, indices, "z",
				minGroundHeight_!=0.0f?minGroundHeight_:std::numeric_limits<int>::min(),
				maxObstacleHeight_>0.0f?maxObstacleHeight_:std::numeric_limits<int>::max());
		UDEBUG("indices after max obstacles height filtering = %d", (int)indices->size());
	}

	if(indices->size())
	{
		if(normalsSegmentation_)
		{
			UDEBUG("normalKSearch=%d", normalKSearch_);
			UDEBUG("maxGroundAngle=%f", maxGroundAngle_);
			UDEBUG("Cluster radius=%f", clusterRadius_);
			UDEBUG("flatObstaclesDetected=%d", flatObstaclesDetected_?1:0);
			UDEBUG("maxGroundHeight=%f", maxGroundHeight_);
			util3d::segmentObstaclesFromGround<PointT>(
					cloud,
					indices,
					groundIndices,
					obstaclesIndices,
					normalKSearch_,
					maxGroundAngle_,
					clusterRadius_,
					minClusterSize_,
					flatObstaclesDetected_,
					maxGroundHeight_,
					flatObstacles,
					Eigen::Vector4f(viewPoint.x, viewPoint.y, viewPoint.z+(projMapFrame_?pose.z():0), 1));
			UDEBUG("viewPoint=%f,%f,%f", viewPoint.x, viewPoint.y, viewPoint.z+(projMapFrame_?pose.z():0));
		}
		else
		{
			UDEBUG("");
			// passthrough filter
			groundIndices = rtabmap::util3d::passThrough(cloud, indices, "z",
					minGroundHeight_!=0.0f?minGroundHeight_:std::numeric_limits<int>::min(),
					maxGroundHeight_!=0.0f?maxGroundHeight_:std::numeric_limits<int>::max());

			pcl::IndicesPtr notObstacles = groundIndices;
			if(indices->size())
			{
				notObstacles = util3d::extractIndices(cloud, indices, true);
				notObstacles = util3d::concatenate(notObstacles, groundIndices);
			}
			obstaclesIndices = rtabmap::util3d::extractIndices(cloud, notObstacles, true);
		}

		UDEBUG("groundIndices=%d obstaclesIndices=%d", (int)groundIndices->size(), (int)obstaclesIndices->size());

		// Do radius filtering after voxel filtering (a lot faster)
		if(noiseFilteringRadius_ > 0.0 && noiseFilteringMinNeighbors_ > 0)
		{
			UDEBUG("");
			if(groundIndices->size())
			{
				groundIndices = rtabmap::util3d::radiusFiltering(cloud, groundIndices, noiseFilteringRadius_, noiseFilteringMinNeighbors_);
			}
			if(obstaclesIndices->size())
			{
				obstaclesIndices = rtabmap::util3d::radiusFiltering(cloud, obstaclesIndices, noiseFilteringRadius_, noiseFilteringMinNeighbors_);
			}
			if(flatObstacles && (*flatObstacles)->size())
			{
				*flatObstacles = rtabmap::util3d::radiusFiltering(cloud, *flatObstacles, noiseFilteringRadius_, noiseFilteringMinNeighbors_);
			}

			if(groundIndices->empty() && obstaclesIndices->empty())
			{
				UWARN("Cloud (with %d points) is empty after noise "
						"filtering. Occupancy grid cannot be "
						"created.",
						(int)cloud->size());

			}
		}
	}
	return cloud;
}

cv::Mat OccupancyGridBuilder::dilate(const cv::Mat& image) const
{
	MEASURE_BLOCK_TIME(OccupancyGrid__dilate);
	UASSERT(semanticDilation_ > 0);
	UASSERT(image.type() == CV_8UC3);
	cv::Mat dilated = cv::Mat::zeros(image.rows, image.cols, image.type());
	for (int h = 0; h < image.rows; h++)
	{
		int dWShift = 0;
		for (int w = 0; w < image.cols; w++)
		{
			const cv::Vec3b& color = image.at<cv::Vec3b>(h, w);
			if (color[0] == 0 && color[1] == 0 && color[2] == 0)
			{
				dWShift = std::max(dWShift - 1, 0);
				continue;
			}
			std::pair<int, int> dHLimits = std::make_pair(
				std::max(h - semanticDilation_, 0),
				std::min(h + semanticDilation_, dilated.rows - 1));
			std::pair<int, int> dWLimits = std::make_pair(
				std::max(w - semanticDilation_ + dWShift, 0),
				std::min(w + semanticDilation_, dilated.cols - 1));
			for (int dH = dHLimits.first; dH <= dHLimits.second; dH++)
			{
				for (int dW = dWLimits.first; dW <= dWLimits.second; dW++)
				{
					cv::Vec3b& dilatedColor = dilated.at<cv::Vec3b>(dH, dW);
					if (dilatedColor[0] != 0 || dilatedColor[1] != 0 || dilatedColor[2] != 0)
					{
						continue;
					}
					dilatedColor = color;
				}
			}
			dWShift = semanticDilation_ * 2;
		}
	}
	lastDilatedSemantic_ = dilated;
	return dilated;
}

LaserScan OccupancyGridBuilder::addSemanticToLaserScan(
		const LaserScan& scan, const std::vector<cv::Mat>& images,
		const std::vector<rtabmap::CameraModel>& cameraModels) const
{
	MEASURE_BLOCK_TIME(OccupancyGrid__addSemanticToLaserScan);
	UASSERT(images.size() == cameraModels.size());
	UASSERT(scan.format() == rtabmap::LaserScan::Format::kXYZ || scan.format() == rtabmap::LaserScan::Format::kXYZI);
	cv::Mat scanRGB_data = cv::Mat(1, scan.size(),
		CV_32FC(rtabmap::LaserScan::channels(rtabmap::LaserScan::Format::kXYZRGB)));
	std::vector<rtabmap::Transform> cameras2LaserScan;
	cameras2LaserScan.reserve(images.size());
	for (int camId = 0; camId < images.size(); camId++)
	{
		cameras2LaserScan.push_back(cameraModels[camId].localTransform().inverse() * scan.localTransform());
	}
	for (int i = 0; i < scan.size(); i++)
	{
		float* ptr = scanRGB_data.ptr<float>(0, i);
		ptr[0] = scan.field(i, 0);
		ptr[1] = scan.field(i, 1);
		ptr[2] = scan.field(i, 2);
		bool foundColor = false;
		for (int camId = 0; camId < images.size(); camId++)
		{
			const cv::Mat& image = images[camId];
			const rtabmap::CameraModel& cameraModel = cameraModels[camId];
			UASSERT(image.type() == CV_8UC3);
			const rtabmap::Transform& camera2LaserScan = cameras2LaserScan[camId];
			cv::Point3f cameraPoint = rtabmap::util3d::transformPoint(*(cv::Point3f*)(scan.data().ptr<float>(0, i)), camera2LaserScan);
			int u, v;
			cameraModel.reproject(cameraPoint.x, cameraPoint.y, cameraPoint.z, u, v);
			float cameraPointRangeSqr = cameraPoint.x * cameraPoint.x + cameraPoint.y * cameraPoint.y +
				cameraPoint.z * cameraPoint.z;
			if (cameraModel.inFrame(u, v) && cameraPoint.z > 0 &&
				(minSemanticRangeSqr_ == 0.0f || cameraPointRangeSqr > minSemanticRangeSqr_) &&
				(maxSemanticRangeSqr_ == 0.0f || cameraPointRangeSqr < maxSemanticRangeSqr_))
			{
				std::uint8_t* ptrColor = (std::uint8_t*)(ptr + 3);
				const std::uint8_t* bgrColor = image.ptr<std::uint8_t>(v, u);
				ptrColor[0] = bgrColor[0];
				ptrColor[1] = bgrColor[1];
				ptrColor[2] = bgrColor[2];
				ptrColor[3] = (std::uint8_t)255;
				foundColor = true;
				break;
			}
		}
		if (!foundColor)
		{
			std::uint32_t* ptrInt = (std::uint32_t*)ptr;
			ptrInt[3] = 0u;
		}
	}

	LaserScan scanRGB = LaserScan(scanRGB_data, scan.maxPoints(), scan.rangeMax(),
		LaserScan::Format::kXYZRGB, scan.localTransform());
	return scanRGB;
}

OccupancyGridBuilder::LocalMap OccupancyGridBuilder::cvMatsToLocalMap(
		const cv::Mat & groundCells,
		const cv::Mat & emptyCells,
		const cv::Mat & obstacleCells) const
{
	MEASURE_BLOCK_TIME(OccupancyGrid__cvMatsToLocalMap);
	LocalMap localMap;
	localMap.points.resize(3, groundCells.cols + emptyCells.cols + obstacleCells.cols);
	localMap.colors.reserve(groundCells.cols + emptyCells.cols + obstacleCells.cols);
	int shift = 0;

	for (int i = 0; i < groundCells.cols; i++)
	{
		const float * point = groundCells.ptr<float>(0, i);
		localMap.points(0, i) = point[0];
		localMap.points(1, i) = point[1];
		localMap.points(2, i) = 0;
		if (groundCells.channels() == 2)
		{
			localMap.colors.push_back(-1);
		}
		else
		{
			int color = *(int*)(point + 2);
			localMap.colors.push_back(color);
		}
	}
	localMap.numGround = groundCells.cols;

	shift = localMap.numGround;
	for (int i = 0; i < emptyCells.cols; i++)
	{
		const float * point = emptyCells.ptr<float>(0, i);
		localMap.points(0, i + shift) = point[0];
		localMap.points(1, i + shift) = point[1];
		localMap.points(2, i + shift) = 0;
		if (emptyCells.channels() == 2)
		{
			localMap.colors.push_back(-1);
		}
		else
		{
			int color = *(int*)(point + 2);
			localMap.colors.push_back(color);
		}
	}
	localMap.numEmpty = emptyCells.cols;

	shift = localMap.numGround + localMap.numEmpty;
	for (int i = 0; i < obstacleCells.cols; i++)
	{
		const float * point = obstacleCells.ptr<float>(0, i);
		localMap.points(0, i + shift) = point[0];
		localMap.points(1, i + shift) = point[1];
		localMap.points(2, i + shift) = 0;
		if (obstacleCells.channels() == 2)
		{
			localMap.colors.push_back(-1);
		}
		else
		{
			int color = *(int*)(point + 2);
			localMap.colors.push_back(color);
		}
	}
	localMap.numObstacles = obstacleCells.cols;

	return localMap;
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
	std::vector<int> nodeIds;
	nodeIds.reserve(map_.nodes.size());
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
	}
	if (newMapLimits.valid())
	{
		createOrResizeMap(temporaryMap_, newMapLimits);
		for (auto& entry : temporaryMap_.nodes)
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
		map.map = Eigen::MatrixXf::Constant(newMapLimits.height(), newMapLimits.width(), 0.0f);
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

		Eigen::MatrixXf newMap = Eigen::MatrixXf::Constant(newMapLimits.height(), newMapLimits.width(), 0.0f);
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
		float occThr = logodds(occupancyThr_);
		int shiftX = map_.mapLimits.minX - occupancyMapLimits.minX;
		int shiftY = map_.mapLimits.minY - occupancyMapLimits.minY;
		for(int x = 0; x < map_.map.cols(); ++x)
		{
			for(int y = 0; y < map_.map.rows(); ++y)
			{
				float logodds = map_.map(y, x);
				if(logodds == 0.0f)
				{
					occupancyGrid(y + shiftY, x + shiftX) = -1;
				}
				else if(logodds >= occThr)
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
		float tmpOccThr = logodds(temporaryOccupancyThr_);
		int shiftX = temporaryMap_.mapLimits.minX - occupancyMapLimits.minX;
		int shiftY = temporaryMap_.mapLimits.minY - occupancyMapLimits.minY;
		for(int x = 0; x < temporaryMap_.missCounter.cols(); ++x)
		{
			for(int y = 0; y < temporaryMap_.missCounter.rows(); ++y)
			{
				float logodds = temporaryMap_.missCounter(y, x) * temporaryProbMiss_ +
					temporaryMap_.hitCounter(y, x) * temporaryProbHit_;
				if(logodds == 0.0f)
				{
					continue;
				}
				else if(logodds >= tmpOccThr)
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
				if(logodds == 0.0f)
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
				float logodds = temporaryMap_.missCounter(y, x) * temporaryProbMiss_ +
					temporaryMap_.hitCounter(y, x) * temporaryProbHit_;
				if(logodds == 0.0f)
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
