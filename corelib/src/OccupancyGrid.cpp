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

#include <rtabmap/core/OccupancyGrid.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>

#ifdef RTABMAP_OCTOMAP
#include <rtabmap/core/OctoMap.h>
#endif

#include <pcl/io/pcd_io.h>

namespace rtabmap {

OccupancyGrid::OccupancyGrid(const ParametersMap & parameters) :
	parameters_(parameters),
	cloudDecimation_(Parameters::defaultGridDepthDecimation()),
	cloudMaxDepth_(Parameters::defaultGridRangeMax()),
	cloudMinDepth_(Parameters::defaultGridRangeMin()),
	//roiRatios_(Parameters::defaultGridDepthRoiRatios()), // initialized in parseParameters()
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
	grid3D_(Parameters::defaultGrid3D()),
	groundIsObstacle_(Parameters::defaultGridGroundIsObstacle()),
	noiseFilteringRadius_(Parameters::defaultGridNoiseFilteringRadius()),
	noiseFilteringMinNeighbors_(Parameters::defaultGridNoiseFilteringMinNeighbors()),
	scan2dUnknownSpaceFilled_(Parameters::defaultGridScan2dUnknownSpaceFilled()),
	rayTracing_(Parameters::defaultGridRayTracing()),
	fullUpdate_(Parameters::defaultGridGlobalFullUpdate()),
	minMapSize_(Parameters::defaultGridGlobalMinSize()),
	erode_(Parameters::defaultGridGlobalEroded()),
	footprintRadius_(Parameters::defaultGridGlobalFootprintRadius()),
	updateError_(Parameters::defaultGridGlobalUpdateError()),
	occupancyThr_(Parameters::defaultGridGlobalOccupancyThr()),
	probHit_(logodds(Parameters::defaultGridGlobalProbHit())),
	probMiss_(logodds(Parameters::defaultGridGlobalProbMiss())),
	probClampingMin_(logodds(Parameters::defaultGridGlobalProbClampingMin())),
	probClampingMax_(logodds(Parameters::defaultGridGlobalProbClampingMax())),
	xMin_(0),
	yMin_(0),
	cloudAssembling_(false),
	assembledGround_(new pcl::PointCloud<pcl::PointXYZRGB>),
	assembledObstacles_(new pcl::PointCloud<pcl::PointXYZRGB>),
	assembledEmptyCells_(new pcl::PointCloud<pcl::PointXYZRGB>)
{
	this->parseParameters(parameters);
}

void OccupancyGrid::parseParameters(const ParametersMap & parameters)
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
	float cellSize = cellSize_;
	if(Parameters::parse(parameters, Parameters::kGridCellSize(), cellSize))
	{
		this->setCellSize(cellSize);
	}

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
	Parameters::parse(parameters, Parameters::kGrid3D(), grid3D_);
	Parameters::parse(parameters, Parameters::kGridGroundIsObstacle(), groundIsObstacle_);
	Parameters::parse(parameters, Parameters::kGridNoiseFilteringRadius(), noiseFilteringRadius_);
	Parameters::parse(parameters, Parameters::kGridNoiseFilteringMinNeighbors(), noiseFilteringMinNeighbors_);
	Parameters::parse(parameters, Parameters::kGridScan2dUnknownSpaceFilled(), scan2dUnknownSpaceFilled_);
	Parameters::parse(parameters, Parameters::kGridRayTracing(), rayTracing_);
	Parameters::parse(parameters, Parameters::kGridGlobalFullUpdate(), fullUpdate_);
	Parameters::parse(parameters, Parameters::kGridGlobalMinSize(), minMapSize_);
	Parameters::parse(parameters, Parameters::kGridGlobalEroded(), erode_);
	Parameters::parse(parameters, Parameters::kGridGlobalFootprintRadius(), footprintRadius_);
	Parameters::parse(parameters, Parameters::kGridGlobalUpdateError(), updateError_);

	Parameters::parse(parameters, Parameters::kGridGlobalOccupancyThr(), occupancyThr_);
	if(Parameters::parse(parameters, Parameters::kGridGlobalProbHit(), probHit_))
	{
		probHit_ = logodds(probHit_);
		UASSERT_MSG(probHit_ >= 0.0f, uFormat("probHit_=%f",probHit_).c_str());
	}
	if(Parameters::parse(parameters, Parameters::kGridGlobalProbMiss(), probMiss_))
	{
		probMiss_ = logodds(probMiss_);
		UASSERT_MSG(probMiss_ <= 0.0f, uFormat("probMiss_=%f",probMiss_).c_str());
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

	UASSERT(minMapSize_ >= 0.0f);

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

void OccupancyGrid::setCellSize(float cellSize)
{
	UASSERT_MSG(cellSize > 0.0f, uFormat("Param name is \"%s\"", Parameters::kGridCellSize().c_str()).c_str());
	if(cellSize_ != cellSize)
	{
		if(!map_.empty())
		{
			UWARN("Grid cell size has changed, the map is cleared!");
		}
		clear();
		cellSize_ = cellSize;
	}
}

void OccupancyGrid::setCloudAssembling(bool enabled)
{
	cloudAssembling_ = enabled;
	if(!cloudAssembling_)
	{
		assembledGround_->clear();
		assembledObstacles_->clear();
	}
}

void OccupancyGrid::createLocalMap(
		const Signature & node,
		cv::Mat & groundCells,
		cv::Mat & obstacleCells,
		cv::Mat & emptyCells,
		cv::Point3f & viewPoint) const
{
	UDEBUG("scan format=%s, occupancyFromDepth_=%d normalsSegmentation_=%d grid3D_=%d",
			node.sensorData().laserScanRaw().isEmpty()?"NA":node.sensorData().laserScanRaw().formatName().c_str(), occupancyFromDepth_?1:0, normalsSegmentation_?1:0, grid3D_?1:0);

	if((node.sensorData().laserScanRaw().is2d()) && !occupancyFromDepth_)
	{
		UDEBUG("2D laser scan");
		//2D
		viewPoint = cv::Point3f(
				node.sensorData().laserScanRaw().localTransform().x(),
				node.sensorData().laserScanRaw().localTransform().y(),
				node.sensorData().laserScanRaw().localTransform().z());

		LaserScan scan = node.sensorData().laserScanRaw();
		if(cloudMinDepth_ > 0.0f)
		{
			scan = util3d::rangeFiltering(scan, cloudMinDepth_, 0.0f);
		}

		float maxRange = cloudMaxDepth_;
		if(cloudMaxDepth_>0.0f && node.sensorData().laserScanRaw().rangeMax()>0.0f)
		{
			maxRange = cloudMaxDepth_ < node.sensorData().laserScanRaw().rangeMax()?cloudMaxDepth_:node.sensorData().laserScanRaw().rangeMax();
		}
		else if(scan2dUnknownSpaceFilled_ && node.sensorData().laserScanRaw().rangeMax()>0.0f)
		{
			maxRange = node.sensorData().laserScanRaw().rangeMax();
		}
		util3d::occupancy2DFromLaserScan(
				util3d::transformLaserScan(scan, node.sensorData().laserScanRaw().localTransform()).data(),
				cv::Mat(),
				viewPoint,
				emptyCells,
				obstacleCells,
				cellSize_,
				scan2dUnknownSpaceFilled_,
				maxRange);

		UDEBUG("ground=%d obstacles=%d channels=%d", emptyCells.cols, obstacleCells.cols, obstacleCells.cols?obstacleCells.channels():emptyCells.channels());
	}
	else
	{
		// 3D
		if(!occupancyFromDepth_)
		{
			if(!node.sensorData().laserScanRaw().isEmpty())
			{
				UDEBUG("3D laser scan");
				const Transform & t = node.sensorData().laserScanRaw().localTransform();
				LaserScan scan = util3d::downsample(node.sensorData().laserScanRaw(), scanDecimation_);
#ifdef RTABMAP_OCTOMAP
				// clipping will be done in OctoMap
				float maxRange = grid3D_&&rayTracing_?0.0f:cloudMaxDepth_;
#else
				float maxRange = cloudMaxDepth_;
#endif
				if(cloudMinDepth_ > 0.0f || maxRange > 0.0f)
				{
					scan = util3d::rangeFiltering(scan, cloudMinDepth_, 0);
				}

				// update viewpoint
				viewPoint = cv::Point3f(t.x(), t.y(), t.z());

				UDEBUG("scan format=%d", scan.format());
				createLocalMap(scan, node.getPose(), groundCells, obstacleCells, emptyCells, viewPoint);
			}
			else
			{
				UWARN("Cannot create local map, scan is empty (node=%d, %s=false).", node.id(), Parameters::kGridFromDepth().c_str());
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
					node.sensorData(),
					cloudDecimation_,
#ifdef RTABMAP_OCTOMAP
					// clipping will be done in OctoMap
					grid3D_&&rayTracing_?0.0f:cloudMaxDepth_,
#else
					cloudMaxDepth_,
#endif
					cloudMinDepth_,
					indices.get(),
					parameters_,
					roiRatios_);

			// update viewpoint
			if(node.sensorData().cameraModels().size())
			{
				// average of all local transforms
				float sum = 0;
				for(unsigned int i=0; i<node.sensorData().cameraModels().size(); ++i)
				{
					const Transform & t = node.sensorData().cameraModels()[i].localTransform();
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
				const Transform & t = node.sensorData().stereoCameraModel().localTransform();
				viewPoint = cv::Point3f(t.x(), t.y(), t.z());
			}
			createLocalMap(LaserScan(util3d::laserScanFromPointCloud(*cloud, indices), 0, 0.0f), node.getPose(), groundCells, obstacleCells, emptyCells, viewPoint);
		}
	}
}

void OccupancyGrid::createLocalMap(
		const LaserScan & scan,
		const Transform & pose,
		cv::Mat & groundCells,
		cv::Mat & obstacleCells,
		cv::Mat & emptyCells,
		cv::Point3f & viewPointInOut) const
{
	if(projMapFrame_)
	{
		//we should rotate viewPoint in /map frame
		float roll, pitch, yaw;
		pose.getEulerAngles(roll, pitch, yaw);
		Transform viewpointRotated = Transform(0,0,0,roll,pitch,0) * Transform(viewPointInOut.x, viewPointInOut.y, viewPointInOut.z, 0,0,0);
		viewPointInOut.x = viewpointRotated.x();
		viewPointInOut.y = viewpointRotated.y();
		viewPointInOut.z = viewpointRotated.z();
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
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudSegmented = segmentCloud<pcl::PointXYZRGBNormal>(cloud, pcl::IndicesPtr(new std::vector<int>), pose, viewPointInOut, groundIndices, obstaclesIndices);
			UDEBUG("groundIndices=%d, obstaclesIndices=%d", (int)groundIndices->size(), (int)obstaclesIndices->size());
			if(grid3D_)
			{
				groundCloud = util3d::laserScanFromPointCloud(*cloudSegmented, groundIndices).data();
				obstaclesCloud = util3d::laserScanFromPointCloud(*cloudSegmented, obstaclesIndices).data();
			}
			else
			{
				util3d::occupancy2DFromGroundObstacles<pcl::PointXYZRGBNormal>(cloudSegmented, groundIndices, obstaclesIndices, groundCells, obstacleCells, cellSize_);
			}
		}
		else if(scan.hasRGB())
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::laserScanToPointCloudRGB(scan, scan.localTransform());
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSegmented = segmentCloud<pcl::PointXYZRGB>(cloud, pcl::IndicesPtr(new std::vector<int>), pose, viewPointInOut, groundIndices, obstaclesIndices);
			UDEBUG("groundIndices=%d, obstaclesIndices=%d", (int)groundIndices->size(), (int)obstaclesIndices->size());
			if(grid3D_)
			{
				groundCloud = util3d::laserScanFromPointCloud(*cloudSegmented, groundIndices).data();
				obstaclesCloud = util3d::laserScanFromPointCloud(*cloudSegmented, obstaclesIndices).data();
			}
			else
			{
				util3d::occupancy2DFromGroundObstacles<pcl::PointXYZRGB>(cloudSegmented, groundIndices, obstaclesIndices, groundCells, obstacleCells, cellSize_);
			}
		}
		else if(scan.hasNormals())
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud = util3d::laserScanToPointCloudNormal(scan, scan.localTransform());
			pcl::PointCloud<pcl::PointNormal>::Ptr cloudSegmented = segmentCloud<pcl::PointNormal>(cloud, pcl::IndicesPtr(new std::vector<int>), pose, viewPointInOut, groundIndices, obstaclesIndices);
			UDEBUG("groundIndices=%d, obstaclesIndices=%d", (int)groundIndices->size(), (int)obstaclesIndices->size());
			if(grid3D_)
			{
				groundCloud = util3d::laserScanFromPointCloud(*cloudSegmented, groundIndices).data();
				obstaclesCloud = util3d::laserScanFromPointCloud(*cloudSegmented, obstaclesIndices).data();
			}
			else
			{
				util3d::occupancy2DFromGroundObstacles<pcl::PointNormal>(cloudSegmented, groundIndices, obstaclesIndices, groundCells, obstacleCells, cellSize_);
			}
		}
		else
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::laserScanToPointCloud(scan, scan.localTransform());
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmented = segmentCloud<pcl::PointXYZ>(cloud, pcl::IndicesPtr(new std::vector<int>), pose, viewPointInOut, groundIndices, obstaclesIndices);
			UDEBUG("groundIndices=%d, obstaclesIndices=%d", (int)groundIndices->size(), (int)obstaclesIndices->size());
			if(grid3D_)
			{
				groundCloud = util3d::laserScanFromPointCloud(*cloudSegmented, groundIndices).data();
				obstaclesCloud = util3d::laserScanFromPointCloud(*cloudSegmented, obstaclesIndices).data();
			}
			else
			{
				util3d::occupancy2DFromGroundObstacles<pcl::PointXYZ>(cloudSegmented, groundIndices, obstaclesIndices, groundCells, obstacleCells, cellSize_);
			}
		}

		if(grid3D_ && (!obstaclesCloud.empty() || !groundCloud.empty()))
		{
			UDEBUG("ground=%d obstacles=%d", groundCloud.cols, obstaclesCloud.cols);
			if(groundIsObstacle_ && !groundCloud.empty())
			{
				if(obstaclesCloud.empty())
				{
					obstaclesCloud = groundCloud;
					groundCloud = cv::Mat();
				}
				else
				{
					UASSERT(obstaclesCloud.type() == groundCloud.type());
					cv::Mat merged(1,obstaclesCloud.cols+groundCloud.cols, obstaclesCloud.type());
					obstaclesCloud.copyTo(merged(cv::Range::all(), cv::Range(0, obstaclesCloud.cols)));
					groundCloud.copyTo(merged(cv::Range::all(), cv::Range(obstaclesCloud.cols, obstaclesCloud.cols+groundCloud.cols)));
				}
			}

			// transform back in base frame
			float roll, pitch, yaw;
			pose.getEulerAngles(roll, pitch, yaw);
			Transform tinv = Transform(0,0, projMapFrame_?pose.z():0, roll, pitch, 0).inverse();

			if(rayTracing_)
			{
#ifdef RTABMAP_OCTOMAP
				if(!groundCloud.empty() || !obstaclesCloud.empty())
				{
					//create local octomap
					ParametersMap params;
					params.insert(ParametersPair(Parameters::kGridCellSize(), uNumber2Str(cellSize_)));
					params.insert(ParametersPair(Parameters::kGridRangeMax(), uNumber2Str(cloudMaxDepth_)));
					params.insert(ParametersPair(Parameters::kGridRayTracing(), uNumber2Str(rayTracing_)));
					OctoMap octomap(params);
					octomap.addToCache(1, groundCloud, obstaclesCloud, cv::Mat(), cv::Point3f(viewPointInOut.x, viewPointInOut.y, viewPointInOut.z));
					std::map<int, Transform> poses;
					poses.insert(std::make_pair(1, Transform::getIdentity()));
					octomap.update(poses);

					pcl::IndicesPtr groundIndices(new std::vector<int>);
					pcl::IndicesPtr obstaclesIndices(new std::vector<int>);
					pcl::IndicesPtr emptyIndices(new std::vector<int>);
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWithRayTracing = octomap.createCloud(0, obstaclesIndices.get(), emptyIndices.get(), groundIndices.get());
					UDEBUG("ground=%d obstacles=%d empty=%d", (int)groundIndices->size(), (int)obstaclesIndices->size(), (int)emptyIndices->size());
					if(scan.hasRGB())
					{
						groundCells = util3d::laserScanFromPointCloud(*cloudWithRayTracing, groundIndices, tinv).data();
						obstacleCells = util3d::laserScanFromPointCloud(*cloudWithRayTracing, obstaclesIndices, tinv).data();
						emptyCells = util3d::laserScanFromPointCloud(*cloudWithRayTracing, emptyIndices, tinv).data();
					}
					else
					{
						pcl::PointCloud<pcl::PointXYZ>::Ptr cloudWithRayTracing2(new pcl::PointCloud<pcl::PointXYZ>);
						pcl::copyPointCloud(*cloudWithRayTracing, *cloudWithRayTracing2);
						groundCells = util3d::laserScanFromPointCloud(*cloudWithRayTracing2, groundIndices, tinv).data();
						obstacleCells = util3d::laserScanFromPointCloud(*cloudWithRayTracing2, obstaclesIndices, tinv).data();
						emptyCells = util3d::laserScanFromPointCloud(*cloudWithRayTracing2, emptyIndices, tinv).data();
					}
				}
			}
			else
#else
					UWARN("RTAB-Map is not built with OctoMap dependency, 3D ray tracing is ignored. Set \"%s\" to false to avoid this warning.", Parameters::kGridRayTracing().c_str());
				}
#endif
			{
				groundCells = util3d::transformLaserScan(LaserScan::backwardCompatibility(groundCloud), tinv).data();
				obstacleCells = util3d::transformLaserScan(LaserScan::backwardCompatibility(obstaclesCloud), tinv).data();
			}

		}
		else if(!grid3D_ && rayTracing_ && (!obstacleCells.empty() || !groundCells.empty()))
		{
			cv::Mat laserScan = obstacleCells;
			cv::Mat laserScanNoHit = groundCells;
			obstacleCells = cv::Mat();
			groundCells = cv::Mat();
			util3d::occupancy2DFromLaserScan(
					laserScan,
					laserScanNoHit,
					viewPointInOut,
					emptyCells,
					obstacleCells,
					cellSize_,
					false, // don't fill unknown space
					cloudMaxDepth_);
		}
	}
	UDEBUG("ground=%d obstacles=%d empty=%d, channels=%d", groundCells.cols, obstacleCells.cols, emptyCells.cols, obstacleCells.cols?obstacleCells.channels():groundCells.channels());
}

void OccupancyGrid::clear()
{
	localMaps_.clear();
	map_ = cv::Mat();
	colors_ = cv::Mat();
	xMin_ = 0;
	yMin_ = 0;
	addedPoses_.clear();
	assembledGround_->clear();
	assembledObstacles_->clear();
}

cv::Mat OccupancyGrid::getMap(float & xMin, float & yMin) const
{
	xMin = xMin_ * cellSize_;
	yMin = yMin_ * cellSize_;

	cv::Mat map;

	UTimer t;
	if(occupancyThr_ != 0.0f && !map_.empty())
	{
		float occThr = logodds(occupancyThr_);
		map = cv::Mat(map_.size(), CV_8SC1);
		for(int i=0; i<map.rows; ++i)
		{
			for(int j=0; j<map.cols; ++j)
			{
				float logodds = map_.at<float>(i, j);
				if(logodds == 0.0f)
				{
					map.at<char>(i, j) = -1; // unknown
				}
				else if(logodds >= occThr)
				{
					map.at<char>(i, j) = 100; // unknown
				}
				else
				{
					map.at<char>(i, j) = 0; // empty
				}
			}
		}
	}

	for (const auto& pair : temporarilyOccupiedCells_)
	{
		int x = pair.first;
		int y = pair.second;
		map.at<char>(y, x) = 100;
	}

	if(erode_ && !map.empty())
	{
		map = util3d::erodeMap(map);
	}
	return map;
}

cv::Mat OccupancyGrid::getProbMap(float & xMin, float & yMin) const
{
	xMin = xMin_ * cellSize_;
	yMin = yMin_ * cellSize_;

	cv::Mat map;

	UTimer t;
	if(occupancyThr_ != 0.0f && !map.empty())
	{
		float occThr = logodds(occupancyThr_);
		map = cv::Mat(map.size(), CV_8SC1);
		for(int i=0; i<map.rows; ++i)
		{
			for(int j=0; j<map.cols; ++j)
			{
				float logodds = map_.at<float>(i, j);
				if(logodds == 0.0f)
				{
					map.at<char>(i, j) = -1; // unknown
				}
				else if(logodds >= occThr)
				{
					map.at<char>(i, j) = probability(logodds);
				}
			}
		}
	}

	for (const auto& pair : temporarilyOccupiedCells_)
	{
		int x = pair.first;
		int y = pair.second;
		map.at<char>(y, x) = 100;
	}

	return map;
}

cv::Mat OccupancyGrid::getColors(float & xMin, float & yMin) const
{
	xMin = xMin_ * cellSize_;
	yMin = yMin_ * cellSize_;

	cv::Mat colors;
	if(!colors_.empty())
	{
		colors = colors_.clone();
	}
	else
	{
		UWARN("Colors are empty");
	}
	return colors;
}

void OccupancyGrid::addToCache(
		int nodeId,
		const cv::Mat & ground,
		const cv::Mat & obstacles,
		const cv::Mat & empty)
{
	OccupancyGrid::LocalMap localMap;
	localMap.points.resize(3, ground.cols + empty.cols + obstacles.cols);
	localMap.transformedPoints2d.resize(2, 0);
	localMap.colors.reserve(ground.cols + empty.cols + obstacles.cols);
	int shift = 0;

	for (int i = 0; i < ground.cols; i++)
	{
		const float * point = ground.ptr<float>(0, i);
		localMap.points(0, i) = point[0];
		localMap.points(1, i) = point[1];
		localMap.points(2, i) = 0;
		if (ground.channels() == 2)
		{
			localMap.colors.push_back(-1);
		}
		else
		{
			int color = *(int*)(point + 2);
			if (color == 0)
			{
				localMap.colors.push_back(-1);
			}
			else
			{
				localMap.colors.push_back(color);
			}
		}
	}
	localMap.numGround = ground.cols;

	shift = localMap.numGround;
	for (int i = 0; i < empty.cols; i++)
	{
		const float * point = empty.ptr<float>(0, i);
		localMap.points(0, i + shift) = point[0];
		localMap.points(1, i + shift) = point[1];
		localMap.points(2, i + shift) = 0;
		if (empty.channels() == 2)
		{
			localMap.colors.push_back(-1);
		}
		else
		{
			int color = *(int*)(point + 2);
			if (color == 0)
			{
				localMap.colors.push_back(-1);
			}
			else
			{
				localMap.colors.push_back(color);
			}
		}
	}
	localMap.numEmpty = empty.cols;

	shift = localMap.numGround + localMap.numEmpty;
	for (int i = 0; i < obstacles.cols; i++)
	{
		const float * point = obstacles.ptr<float>(0, i);
		localMap.points(0, i + shift) = point[0];
		localMap.points(1, i + shift) = point[1];
		localMap.points(2, i + shift) = 0;
		if (obstacles.channels() == 2)
		{
			localMap.colors.push_back(-1);
		}
		else
		{
			int color = *(int*)(point + 2);
			if (color == 0)
			{
				localMap.colors.push_back(-1);
			}
			else
			{
				localMap.colors.push_back(color);
			}
		}
	}
	localMap.numObstacles = obstacles.cols;

	localMaps_[nodeId] = std::move(localMap);
}

void OccupancyGrid::addToCache(
			int nodeId,
			int numGround,
			int numEmpty,
			int numObstacles,
			const Eigen::Matrix3Xf & points,
			const std::vector<int> & colors)
{
	UASSERT(numGround + numEmpty + numObstacles == points.cols());
	OccupancyGrid::LocalMap localMap;
	localMap.numGround = numGround;
	localMap.numEmpty = numEmpty;
	localMap.numObstacles = numObstacles;
	localMap.points = points;
	localMap.colors = colors;
	localMaps_[nodeId] = std::move(localMap);
}

void OccupancyGrid::cacheCurrentMap()
{
	OccupancyGrid::CachedMap * cachedMap = new OccupancyGrid::CachedMap();
	cachedMap->map = map_.clone();
	cachedMap->xMin = xMin_;
	cachedMap->yMin = yMin_;
	cachedMap->colors = colors_.clone();
	cachedMap->poses = addedPoses_;
	cachedMap_.reset(cachedMap);
}

bool OccupancyGrid::tryToUseCachedMap(const std::map<int, Transform> & poses)
{
	if (cachedMap_ == nullptr)
	{
		return false;
	}

	auto it = poses.begin();
	const auto & cachedPoses = cachedMap_->poses;
	auto cachedIt = cachedPoses.begin();
	while (cachedIt != cachedPoses.end())
	{
		if (it == poses.end())
		{
			return false;
		}
		if (*it != *cachedIt)
		{
			return false;
		}
		++it;
		++cachedIt;
	}

	map_ = cachedMap_->map.clone();
	xMin_ = cachedMap_->xMin;
	yMin_ = cachedMap_->yMin;
	colors_ = cachedMap_->colors.clone();
	addedPoses_ = cachedMap_->poses;
	return true;
}

bool OccupancyGrid::update(const std::map<int, Transform> & poses)
{
	UTimer timer;

	bool graphChanged = checkIfGraphChanged(poses);

	if(graphChanged)
	{
		map_ = cv::Mat();
		addedPoses_.clear();
		bool usedCachedMap = tryToUseCachedMap(poses);
	}

	std::vector<int> newNodeIds;
	newNodeIds.reserve(poses.size() - addedPoses_.size());
	auto from = poses.begin();
	if(!addedPoses_.empty())
	{
		from = poses.lower_bound(addedPoses_.rbegin()->first + 1);
	}
	for(auto iter=from; iter!=poses.end(); ++iter)
	{
		newNodeIds.push_back(iter->first);
		addedPoses_.insert(*iter);
	}

	transformLocalMaps(newNodeIds);

	int xMin, yMin, xMax, yMax;
	getNewMapSize(newNodeIds, xMin, yMin, xMax, yMax);
	createOrExtendMapIfNeeded(xMin, yMin, xMax, yMax);

	for(auto newNodeId : newNodeIds)
	{
		deployLocalMap(newNodeId);
	}

	return graphChanged;
}

bool OccupancyGrid::checkIfGraphChanged(const std::map<int, Transform> & poses)
{
	if(addedPoses_.empty())
	{
		return false;
	}

	bool graphChanged = false;
	int posesCounter = 0;
	for(auto iter=addedPoses_.begin(); iter!=addedPoses_.end(); ++iter)
	{
		auto jter = poses.find(iter->first);
		if(jter != poses.end())
		{
			if(iter->second != jter->second)
			{
				graphChanged = true;
				break;
			}
			posesCounter++;
		}
		else
		{
			graphChanged = true;
			break;
		}
	}
	if(!graphChanged)
	{
		for(auto iter=poses.lower_bound(addedPoses_.rbegin()->first + 1); iter!=poses.end(); ++iter)
		{
			posesCounter++;
		}
		if(posesCounter != poses.size())
		{
			graphChanged = true;
		}
	}
	return graphChanged;
}

void OccupancyGrid::transformLocalMap(int nodeId)
{
	OccupancyGrid::LocalMap & localMap = localMaps_.at(nodeId);
	localMap.transformedPoints2d.resize(2, localMap.points.cols());

	const Transform & pose = addedPoses_.at(nodeId);
	Eigen::Matrix3Xf transformedPoints = (pose.toEigen3fRotation() * localMap.points).colwise() + pose.toEigen3fTranslation();
	for (int i = 0; i < transformedPoints.cols(); i++)
	{
		localMap.transformedPoints2d(0, i) = std::floor(transformedPoints(0, i) / cellSize_);
		localMap.transformedPoints2d(1, i) = std::floor(transformedPoints(1, i) / cellSize_);
	}
}

void OccupancyGrid::transformLocalMaps(const std::vector<int> & nodeIds)
{
	for(auto nodeId : nodeIds)
	{
		transformLocalMap(nodeId);
	}
}

bool OccupancyGrid::isLocalMapTransformed(int nodeId)
{
	return localMaps_.at(nodeId).transformedPoints2d.cols() != 0;
}

void OccupancyGrid::getNewMapSize(const std::vector<int> & newNodeIds, int & xMin, int & yMin, int & xMax, int & yMax)
{
	bool undefinedSize = true;
	const int border = 10;
	if(!map_.empty())
	{
		xMin = xMin_ + border;
		yMin = yMin_ + border;
		xMax = xMin_ + map_.cols - 1 - border;
		yMax = yMin_ + map_.rows - 1 - border;
		undefinedSize = false;
	}
	for(int nodeId : newNodeIds)
	{
		UASSERT(isLocalMapTransformed(nodeId));
		const OccupancyGrid::LocalMap & localMap = localMaps_.at(nodeId);
		for (int i = 0; i < localMap.points.cols(); i++)
		{
			int x = localMap.transformedPoints2d(0, i);
			int y = localMap.transformedPoints2d(1, i);

			if(undefinedSize)
			{
				xMin = x;
				yMin = y;
				xMax = x;
				yMax = y;
				undefinedSize = false;
			}

			if(xMin > x)
				xMin = x;
			else if(xMax < x)
				xMax = x;

			if(yMin > y)
				yMin = y;
			else if(yMax < y)
				yMax = y;
		}
	}

	xMin -= border;
	yMin -= border;
	xMax += border;
	yMax += border;
}

void OccupancyGrid::createOrExtendMapIfNeeded(int xMin, int yMin, int xMax, int yMax)
{
	cv::Size newMapSize(xMax - xMin + 1, yMax - yMin + 1);
	if(map_.empty())
	{
		map_ = cv::Mat::zeros(newMapSize, CV_32FC1);
		colors_ = cv::Mat::zeros(newMapSize, CV_8UC3);
		xMin_ = xMin;
		yMin_ = yMin;
	}
	else if(xMin != xMin_ || yMin != yMin_ ||
			newMapSize.width != map_.cols ||
			newMapSize.height != map_.rows)
	{
		int deltaX = xMin_ - xMin;
		int deltaY = yMin_ - yMin;

		cv::Mat newMap = cv::Mat::zeros(newMapSize, CV_32FC1);
		cv::Mat newColors = cv::Mat::zeros(newMapSize, CV_8UC3);

		map_.copyTo(newMap(cv::Rect(deltaX, deltaY, map_.cols, map_.rows)));
		colors_.copyTo(newColors(cv::Rect(deltaX, deltaY, map_.cols, map_.rows)));

		map_ = newMap;
		colors_ = newColors;
		xMin_ = xMin;
		yMin_ = yMin;
	}
}

void OccupancyGrid::deployLocalMap(int nodeId)
{
	UASSERT(isLocalMapTransformed(nodeId));
	temporarilyOccupiedCells_.clear();
	const OccupancyGrid::LocalMap & localMap = localMaps_.at(nodeId);
	for (int i = 0; i < localMap.numGround + localMap.numEmpty; i++)
	{
		int x = localMap.transformedPoints2d(0, i) - xMin_;
		int y = localMap.transformedPoints2d(1, i) - yMin_;
		UASSERT(y >= 0 && y < map_.rows && x >= 0 && x < map_.cols);

		float & logodds = map_.at<float>(y, x);
		logodds += probMiss_;
		if (logodds < probClampingMin_)
		{
			logodds = probClampingMin_;
		}

		const auto & colors = localMap.colors;
		cv::Vec3b & color = colors_.at<cv::Vec3b>(y, x);
		int c = colors[i];
		if (c != -1)
		{
			color[0] = (unsigned char)(c & 0xFF);
			color[1] = (unsigned char)((c >> 8) & 0xFF);
			color[2] = (unsigned char)((c >> 16) & 0xFF);
		}
	}

	for (int i = localMap.numGround + localMap.numEmpty; i < localMap.points.cols(); i++)
	{
		int x = localMap.transformedPoints2d(0, i) - xMin_;
		int y = localMap.transformedPoints2d(1, i) - yMin_;
		UASSERT(y >= 0 && y < map_.rows && x >= 0 && x < map_.cols);

		{
			const auto & colors = localMap.colors;
			int c = colors[i];
			if (c == personColor_)
			{
				temporarilyOccupiedCells_.emplace_back(x, y);
				continue;
			}
		}

		float & logodds = map_.at<float>(y, x);
		logodds += probHit_;
		if (logodds > probClampingMax_)
		{
			logodds = probClampingMax_;
		}

		const auto & colors = localMap.colors;
		cv::Vec3b & color = colors_.at<cv::Vec3b>(y, x);
		int c = colors[i];
		if (c != -1)
		{
			color[0] = (unsigned char)(c & 0xFF);
			color[1] = (unsigned char)((c >> 8) & 0xFF);
			color[2] = (unsigned char)((c >> 16) & 0xFF);
		}
	}
}

unsigned long OccupancyGrid::getMemoryUsed() const
{
	// unsigned long memoryUsage = sizeof(OccupancyGrid);
	// memoryUsage += parameters_.size()*(sizeof(std::string)*2+sizeof(ParametersMap::iterator)) + sizeof(ParametersMap);

	// memoryUsage += localMaps_.size()*(sizeof(int) + sizeof(std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat>) + sizeof(std::map<int, std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat> >::iterator)) + sizeof(std::map<int, std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat> >);
	// for(std::map<int, std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat> >::const_iterator iter=localMaps_.begin(); iter!=localMaps_.end(); ++iter)
	// {
	// 	memoryUsage += iter->second.first.first.total() * iter->second.first.first.elemSize();
	// 	memoryUsage += iter->second.first.second.total() * iter->second.first.second.elemSize();
	// 	memoryUsage += iter->second.second.total() * iter->second.second.elemSize();
	// }
	// memoryUsage += map_.total() * map_.elemSize();
	// memoryUsage += addedPoses_.size()*(sizeof(int) + sizeof(Transform)+ sizeof(float)*12 + sizeof(std::map<int, Transform>::iterator)) + sizeof(std::map<int, Transform>);

	// if(assembledGround_.get())
	// {
	// 	memoryUsage += assembledGround_->points.size() * sizeof(pcl::PointXYZRGB);
	// }
	// if(assembledObstacles_.get())
	// {
	// 	memoryUsage += assembledObstacles_->points.size() * sizeof(pcl::PointXYZRGB);
	// }
	// if(assembledEmptyCells_.get())
	// {
	// 	memoryUsage += assembledEmptyCells_->points.size() * sizeof(pcl::PointXYZRGB);
	// }
	// return memoryUsage;
}

}
