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
#include <rtabmap/core/util3d_mapping.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>

#include <pcl/io/pcd_io.h>

namespace rtabmap {

OccupancyGrid::OccupancyGrid(const ParametersMap & parameters) :
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
	erode_(Parameters::defaultGridGlobalEroded()),
	footprintRadius_(Parameters::defaultGridGlobalFootprintRadius()),
	occupancyThr_(Parameters::defaultGridGlobalOccupancyThr()),
	probHit_(logodds(Parameters::defaultGridGlobalProbHit())),
	probMiss_(logodds(Parameters::defaultGridGlobalProbMiss())),
	probClampingMin_(logodds(Parameters::defaultGridGlobalProbClampingMin())),
	probClampingMax_(logodds(Parameters::defaultGridGlobalProbClampingMax())),
	minSemanticRange_(Parameters::defaultGridMinSemanticRange()),
	maxSemanticRange_(Parameters::defaultGridMaxSemanticRange()),
	temporarilyOccupiedCellsColor_(Parameters::defaultGridTemporarilyOccupiedCellsColor()),
	showTemporarilyOccupiedCells_(Parameters::defaultGridShowTemporarilyOccupiedCells()),
	xMin_(0),
	yMin_(0)
{
	parseParameters(parameters);
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
	Parameters::parse(parameters, Parameters::kGridGlobalEroded(), erode_);
	Parameters::parse(parameters, Parameters::kGridGlobalFootprintRadius(), footprintRadius_);

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

	Parameters::parse(parameters, Parameters::kGridTemporarilyOccupiedCellsColor(), temporarilyOccupiedCellsColor_);
	Parameters::parse(parameters, Parameters::kGridShowTemporarilyOccupiedCells(), showTemporarilyOccupiedCells_);

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

OccupancyGrid::LocalMap OccupancyGrid::createLocalMap(const Signature & signature) const
{
	cv::Mat groundCells;
	cv::Mat emptyCells;
	cv::Mat obstacleCells;
	cv::Point3f viewPoint;

	if(signature.sensorData().laserScanRaw().is2d() && !occupancyFromDepth_)
	{
		UDEBUG("2D laser scan");
		//2D
		viewPoint = cv::Point3f(
				signature.sensorData().laserScanRaw().localTransform().x(),
				signature.sensorData().laserScanRaw().localTransform().y(),
				signature.sensorData().laserScanRaw().localTransform().z());

		LaserScan scan = signature.sensorData().laserScanRaw();
		if(cloudMinDepth_ > 0.0f)
		{
			scan = util3d::rangeFiltering(scan, cloudMinDepth_, 0.0f);
		}

		float maxRange = cloudMaxDepth_;
		if(cloudMaxDepth_>0.0f && signature.sensorData().laserScanRaw().rangeMax()>0.0f)
		{
			maxRange = cloudMaxDepth_ < signature.sensorData().laserScanRaw().rangeMax()?cloudMaxDepth_:signature.sensorData().laserScanRaw().rangeMax();
		}
		else if(scan2dUnknownSpaceFilled_ && signature.sensorData().laserScanRaw().rangeMax()>0.0f)
		{
			maxRange = signature.sensorData().laserScanRaw().rangeMax();
		}
		util3d::occupancy2DFromLaserScan(
				util3d::transformLaserScan(scan, signature.sensorData().laserScanRaw().localTransform()).data(),
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
			if(!signature.sensorData().laserScanRaw().isEmpty())
			{
				UDEBUG("3D laser scan");
				const Transform & t = signature.sensorData().laserScanRaw().localTransform();
				LaserScan scan = util3d::downsample(signature.sensorData().laserScanRaw(), scanDecimation_);
				if(cloudMinDepth_ > 0.0f || cloudMaxDepth_ > 0.0f)
				{
					scan = util3d::rangeFiltering(scan, cloudMinDepth_, 0);
				}

				if(!signature.sensorData().imageRaw().empty() && signature.sensorData().cameraModels().size())
				{
					scan = addSemanticToLaserScan(scan, signature.sensorData().imageRaw(), signature.sensorData().cameraModels());
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

LaserScan OccupancyGrid::addSemanticToLaserScan(
		const LaserScan& scan, const cv::Mat& rgb,
		const std::vector<rtabmap::CameraModel>& cameraModels) const
{
	cv::Mat scanRGB_data = cv::Mat(1, scan.size(),
		CV_32FC(rtabmap::LaserScan::channels(rtabmap::LaserScan::Format::kXYZRGB)));
	UASSERT(scan.format() == rtabmap::LaserScan::Format::kXYZ || scan.format() == rtabmap::LaserScan::Format::kXYZI);
	UASSERT(rgb.type() == CV_8UC3);
	rtabmap::Transform camera2LaserScan = cameraModels[0].localTransform().inverse() * scan.localTransform();
	for (int i = 0; i < scan.size(); i++)
	{
		float* ptr = scanRGB_data.ptr<float>(0, i);
		ptr[0] = scan.field(i, 0);
		ptr[1] = scan.field(i, 1);
		ptr[2] = scan.field(i, 2);

		cv::Point3f cameraPoint = rtabmap::util3d::transformPoint(*(cv::Point3f*)(scan.data().ptr<float>(0, i)), camera2LaserScan);
		int u, v;
		cameraModels[0].reproject(cameraPoint.x, cameraPoint.y, cameraPoint.z, u, v);
		float cameraPointRangeSqr = cameraPoint.x * cameraPoint.x + cameraPoint.y * cameraPoint.y +
			cameraPoint.z * cameraPoint.z;
		if (cameraModels[0].inFrame(u, v) && cameraPoint.z > 0 &&
			(minSemanticRangeSqr_ == 0.0f || cameraPointRangeSqr > minSemanticRangeSqr_) &&
			(maxSemanticRangeSqr_ == 0.0f || cameraPointRangeSqr < maxSemanticRangeSqr_))
		{
			int* ptrInt = (int*)ptr;
			std::uint8_t b, g, r;
			const std::uint8_t* bgrColor = rgb.ptr<std::uint8_t>(v, u);
			b = std::max(bgrColor[0], (std::uint8_t)1);
			g = std::max(bgrColor[1], (std::uint8_t)1);
			r = std::max(bgrColor[2], (std::uint8_t)1);
			ptrInt[3] = int(b) | (int(g) << 8) | (int(r) << 16);

			pcl::PointXYZRGB coloredPoint;
			coloredPoint.x = ptr[0];
			coloredPoint.y = ptr[1];
			coloredPoint.z = ptr[2];
			coloredPoint.r = r;
			coloredPoint.g = g;
			coloredPoint.b = b;
		}
		else
		{
			int* ptrInt = (int*)ptr;
			ptrInt[3] = 0;
		}
	}

	LaserScan scanRGB = LaserScan(scanRGB_data, scan.maxPoints(), scan.rangeMax(),
		LaserScan::Format::kXYZRGB, scan.localTransform());
	return scanRGB;
}

void OccupancyGrid::createLocalMap(
		const LaserScan & scan,
		const Transform & pose,
		cv::Mat & groundCells,
		cv::Mat & emptyCells,
		cv::Mat & obstacleCells,
		cv::Point3f & viewPoint) const
{
	if(projMapFrame_)
	{
		//we should rotate viewPoint in /map frame
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
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::laserScanToPointCloudRGB(scan, scan.localTransform());
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudSegmented = segmentCloud<pcl::PointXYZRGB>(cloud, pcl::IndicesPtr(new std::vector<int>), pose, viewPoint, groundIndices, obstaclesIndices);
			UDEBUG("groundIndices=%d, obstaclesIndices=%d", (int)groundIndices->size(), (int)obstaclesIndices->size());
			util3d::occupancy2DFromGroundObstacles<pcl::PointXYZRGB>(cloudSegmented, groundIndices, obstaclesIndices, groundCells, obstacleCells, cellSize_);
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
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::laserScanToPointCloud(scan, scan.localTransform());
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmented = segmentCloud<pcl::PointXYZ>(cloud, pcl::IndicesPtr(new std::vector<int>), pose, viewPoint, groundIndices, obstaclesIndices);
			UDEBUG("groundIndices=%d, obstaclesIndices=%d", (int)groundIndices->size(), (int)obstaclesIndices->size());
			util3d::occupancy2DFromGroundObstacles<pcl::PointXYZ>(cloudSegmented, groundIndices, obstaclesIndices, groundCells, obstacleCells, cellSize_);
		}

		if(rayTracing_ && (!obstacleCells.empty() || !groundCells.empty()))
		{
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
typename pcl::PointCloud<PointT>::Ptr OccupancyGrid::segmentCloud(
		const typename pcl::PointCloud<PointT>::Ptr & cloudIn,
		const pcl::IndicesPtr & indicesIn,
		const Transform & pose,
		const cv::Point3f & viewPoint,
		pcl::IndicesPtr & groundIndices,
		pcl::IndicesPtr & obstaclesIndices,
		pcl::IndicesPtr * flatObstacles /* nullptr */) const
{
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
			//UWARN("Saving ground.pcd and obstacles.pcd");
			//pcl::io::savePCDFile("ground.pcd", *cloud, *groundIndices);
			//pcl::io::savePCDFile("obstacles.pcd", *cloud, *obstaclesIndices);
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

OccupancyGrid::LocalMap OccupancyGrid::cvMatsToLocalMap(
		const cv::Mat & groundCells,
		const cv::Mat & emptyCells,
		const cv::Mat & obstacleCells) const
{
	OccupancyGrid::LocalMap localMap;
	localMap.points.resize(3, groundCells.cols + emptyCells.cols + obstacleCells.cols);
	localMap.transformedPoints2d.resize(2, 0);
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
	localMap.numObstacles = obstacleCells.cols;

	return localMap;
}

void OccupancyGrid::addLocalMap(int nodeId, OccupancyGrid::LocalMap localMap)
{
	UASSERT(localMap.numGround + localMap.numEmpty + localMap.numObstacles == localMap.points.cols());
	localMaps_[nodeId] = std::move(localMap);
}

void OccupancyGrid::addTemporaryLocalMap(const Transform & temporaryPose, OccupancyGrid::LocalMap temporaryLocalMap)
{
	poses_[-1] = temporaryPose;
	addLocalMap(-1, std::move(temporaryLocalMap));
	transformLocalMap(-1);
	temporaryLocalMap_ = std::make_unique<OccupancyGrid::LocalMap>(std::move(localMaps_.at(-1)));
	poses_.erase(-1);
	localMaps_.erase(-1);
}

void OccupancyGrid::cacheCurrentMap()
{
	cachedMap_ = std::make_unique<OccupancyGrid::CachedMap>();
	cachedMap_->poses = poses_;
	cachedMap_->xMin = xMin_;
	cachedMap_->yMin = yMin_;
	cachedMap_->map = map_.clone();
	cachedMap_->colors = colors_.clone();
	cachedMap_->temporarilyOccupiedCells = temporarilyOccupiedCells_;
}

bool OccupancyGrid::tryToUseCachedMap(const std::map<int, Transform> & updatedPoses)
{
	if (cachedMap_ == nullptr)
	{
		return false;
	}

	auto it = updatedPoses.begin();
	const auto & cachedPoses = cachedMap_->poses;
	auto cachedIt = cachedPoses.begin();
	while (cachedIt != cachedPoses.end())
	{
		if (it == updatedPoses.end())
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

	poses_ = cachedMap_->poses;
	xMin_ = cachedMap_->xMin;
	yMin_ = cachedMap_->yMin;
	map_ = cachedMap_->map.clone();
	colors_ = cachedMap_->colors.clone();
	temporarilyOccupiedCells_ = cachedMap_->temporarilyOccupiedCells;
	return true;
}

bool OccupancyGrid::update(const std::map<int, Transform> & updatedPoses)
{
	temporaryLocalMap_.reset();

	bool graphChanged = checkIfGraphChanged(updatedPoses);

	if(graphChanged)
	{
		map_ = cv::Mat();
		poses_.clear();
		bool usedCachedMap = tryToUseCachedMap(updatedPoses);
	}

	std::vector<int> newNodeIds;
	newNodeIds.reserve(updatedPoses.size() - poses_.size());
	auto from = updatedPoses.begin();
	if(!poses_.empty())
	{
		from = updatedPoses.lower_bound(poses_.rbegin()->first + 1);
	}
	for(auto iter=from; iter!=updatedPoses.end(); ++iter)
	{
		newNodeIds.push_back(iter->first);
		poses_.insert(*iter);
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

bool OccupancyGrid::checkIfGraphChanged(const std::map<int, Transform> & updatedPoses)
{
	if(poses_.empty())
	{
		return false;
	}

	bool graphChanged = false;
	int posesCounter = 0;
	for(auto iter=poses_.begin(); iter!=poses_.end(); ++iter)
	{
		auto jter = updatedPoses.find(iter->first);
		if(jter != updatedPoses.end())
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
		for(auto iter=updatedPoses.lower_bound(poses_.rbegin()->first + 1); iter!=updatedPoses.end(); ++iter)
		{
			posesCounter++;
		}
		if(posesCounter != updatedPoses.size())
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

	const Transform & pose = poses_.at(nodeId);
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
		xMin_ = xMin;
		yMin_ = yMin;
		map_ = cv::Mat::zeros(newMapSize, CV_32FC1);
		colors_ = cv::Mat::zeros(newMapSize, CV_8UC3);
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

		xMin_ = xMin;
		yMin_ = yMin;
		map_ = newMap;
		colors_ = newColors;
	}
}

void OccupancyGrid::deployLocalMap(int nodeId)
{
	UASSERT(isLocalMapTransformed(nodeId));
	temporarilyOccupiedCells_.clear();
	const OccupancyGrid::LocalMap & localMap = localMaps_.at(nodeId);
	for (int i = 0; i < localMap.points.cols(); i++)
	{
		int x = localMap.transformedPoints2d(0, i) - xMin_;
		int y = localMap.transformedPoints2d(1, i) - yMin_;
		UASSERT(y >= 0 && y < map_.rows && x >= 0 && x < map_.cols);

		bool free = (i < localMap.numGround + localMap.numEmpty);
		if (free)
		{
			float & logodds = map_.at<float>(y, x);
			logodds += probMiss_;
			if (logodds < probClampingMin_)
			{
				logodds = probClampingMin_;
			}
		}
		else
		{
			if (temporarilyOccupiedCellsColor_ >= 0)
			{
				const auto & colors = localMap.colors;
				int c = colors[i];
				if (c == temporarilyOccupiedCellsColor_)
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
		}

		int localMapColor = localMap.colors[i];
		if (localMapColor != -1)
		{
			cv::Vec3b & color = colors_.at<cv::Vec3b>(y, x);
			color[0] = (unsigned char)(localMapColor & 0xFF);
			color[1] = (unsigned char)((localMapColor >> 8) & 0xFF);
			color[2] = (unsigned char)((localMapColor >> 16) & 0xFF);
		}
	}
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

	addTemporaryInfoOnMap(map);

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

	addTemporaryInfoOnMap(map);

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

	addTemporaryInfoOnColors(colors);

	return colors;
}

void OccupancyGrid::addTemporaryInfoOnMap(cv::Mat map) const
{
	if (showTemporarilyOccupiedCells_)
	{
		for (const auto& pair : temporarilyOccupiedCells_)
		{
			int x = pair.first;
			int y = pair.second;
			map.at<char>(y, x) = 100;
		}
	}

	if (temporaryLocalMap_)
	{
		for (int i = 0; i < temporaryLocalMap_->points.cols(); i++)
		{
			int x = temporaryLocalMap_->transformedPoints2d(0, i) - xMin_;
			int y = temporaryLocalMap_->transformedPoints2d(1, i) - yMin_;
			if (!(y >= 0 && y < map.rows && x >= 0 && x < map.cols))
			{
				continue;
			}

			bool free = (i < temporaryLocalMap_->numGround + temporaryLocalMap_->numEmpty);
			if (free)
			{
				map.at<char>(y, x) = 0;
			}
			else
			{
				map.at<char>(y, x) = 100;
			}
		}
	}
}

void OccupancyGrid::addTemporaryInfoOnColors(cv::Mat colors) const
{
	if (showTemporarilyOccupiedCells_)
	{
		for (const auto& pair : temporarilyOccupiedCells_)
		{
			int x = pair.first;
			int y = pair.second;
			cv::Vec3b & color = colors.at<cv::Vec3b>(y, x);
			color[0] = (unsigned char)(temporarilyOccupiedCellsColor_ & 0xFF);
			color[1] = (unsigned char)((temporarilyOccupiedCellsColor_ >> 8) & 0xFF);
			color[2] = (unsigned char)((temporarilyOccupiedCellsColor_ >> 16) & 0xFF);
		}
	}

	if (temporaryLocalMap_)
	{
		for (int i = temporaryLocalMap_->numGround + temporaryLocalMap_->numEmpty; i < temporaryLocalMap_->points.cols(); i++)
		{
			int x = temporaryLocalMap_->transformedPoints2d(0, i) - xMin_;
			int y = temporaryLocalMap_->transformedPoints2d(1, i) - yMin_;
			if (!(y >= 0 && y < colors.rows && x >= 0 && x < colors.cols))
			{
				continue;
			}

			const auto & temporaryLocalMapColors = temporaryLocalMap_->colors;
			int temporaryLocalMapColor = temporaryLocalMapColors[i];
			if (temporaryLocalMapColor != -1)
			{
				cv::Vec3b & color = colors.at<cv::Vec3b>(y, x);
				color[0] = (unsigned char)(temporaryLocalMapColor & 0xFF);
				color[1] = (unsigned char)((temporaryLocalMapColor >> 8) & 0xFF);
				color[2] = (unsigned char)((temporaryLocalMapColor >> 16) & 0xFF);
			}
		}
	}
}

float OccupancyGrid::getCellSize() const
{
	return cellSize_;
}

int OccupancyGrid::localMapsNum() const
{
	return localMaps_.size();
}

const std::map<int, OccupancyGrid::LocalMap> & OccupancyGrid::localMaps()
{
	return localMaps_;
}

void OccupancyGrid::clear()
{
	localMaps_.clear();
	poses_.clear();

	xMin_ = 0;
	yMin_ = 0;
	map_ = cv::Mat();
	colors_ = cv::Mat();

	cachedMap_.reset();

	temporarilyOccupiedCells_.clear();
	temporaryLocalMap_.reset();
}

}
