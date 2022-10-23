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

#ifndef CORELIB_SRC_OCCUPANCYGRID_H_
#define CORELIB_SRC_OCCUPANCYGRID_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Signature.h>

#include <memory>

namespace rtabmap {

class RTABMAP_EXP OccupancyGrid
{
public:
	inline static float logodds(double probability)
	{
		return (float) log(probability/(1-probability));
	}

	inline static double probability(double logodds)
	{
		return 1. - ( 1. / (1. + exp(logodds)));
	}

	struct LocalMap
	{
		int numGround, numEmpty, numObstacles;
		Eigen::Matrix3Xf points;
		Eigen::Matrix2Xi transformedPoints2d;
		std::vector<int> colors;
	};

	struct CachedMap
	{
		std::map<int, Transform> poses;

		int xMin;
		int yMin;
		cv::Mat map;
		cv::Mat colors;

		std::list<std::pair<int, int>> temporarilyOccupiedCells;
	};

public:
	OccupancyGrid(const ParametersMap & parameters = ParametersMap());
	void parseParameters(const ParametersMap & parameters);

	LocalMap createLocalMap(const Signature & signature) const;
	void addLocalMap(int nodeId, LocalMap localMap);
	void addLocalMap(int nodeId, const Transform & pose, LocalMap localMap);
	void addTemporaryLocalMap(const Transform & temporaryPose, LocalMap temporaryLocalMap);

	void cacheCurrentMap();

	void updatePoses(const std::map<int, Transform> & updatedPoses,
		const std::list<Transform> & updatedTemporaryPoses = std::list<Transform>());

	cv::Mat getMap(float & xMin, float & yMin) const;
	cv::Mat getProbMap(float & xMin, float & yMin) const;
	cv::Mat getColors(float & xMin, float & yMin) const;

	float getCellSize() const;
	int getMaxTemporaryLocalMaps() const;
	int localMapsNum() const;
	const std::map<int, LocalMap> & localMaps();

	void clear();

private:
	void createLocalMap(
			const LaserScan & cloud,
			const Transform & pose,
			cv::Mat & groundCells,
			cv::Mat & emptyCells,
			cv::Mat & obstacleCells,
			cv::Point3f & viewPoint) const;
	template<typename PointT>
	typename pcl::PointCloud<PointT>::Ptr segmentCloud(
			const typename pcl::PointCloud<PointT>::Ptr & cloud,
			const pcl::IndicesPtr & indices,
			const Transform & pose,
			const cv::Point3f & viewPoint,
			pcl::IndicesPtr & groundIndices,
			pcl::IndicesPtr & obstaclesIndices,
			pcl::IndicesPtr * flatObstacles = nullptr) const;

	LaserScan addSemanticToLaserScan(const LaserScan& scan, const cv::Mat& rgb,
			const std::vector<CameraModel>& cameraModels) const;
	LocalMap cvMatsToLocalMap(
			const cv::Mat & groundCells,
			const cv::Mat & emptyCells,
			const cv::Mat & obstacleCells) const;

	bool tryToUseCachedMap(const std::map<int, Transform> & updatedPoses);

	void transformLocalMap(const Transform & pose, LocalMap & localMap);
	bool isLocalMapTransformed(const LocalMap & localMap) const;
	void getNewMapSize(const LocalMap & transformedLocalMap, int & xMin, int & yMin, int & xMax, int & yMax, bool definedSize);
	void createOrExtendMapIfNeeded(int xMin, int yMin, int xMax, int yMax);
	void deployLocalMap(int nodeId);

	void addTemporaryInfoOnMap(cv::Mat map) const;
	void addTemporaryInfoOnColors(cv::Mat colors) const;

	ParametersMap parameters_;
	unsigned int cloudDecimation_;
	float cloudMaxDepth_;
	float cloudMinDepth_;
	std::vector<float> roiRatios_;
	float footprintLength_;
	float footprintWidth_;
	float footprintHeight_;
	int scanDecimation_;
	float cellSize_;
	bool preVoxelFiltering_;
	bool occupancyFromDepth_;
	bool projMapFrame_; // false по умолчанию
	float maxObstacleHeight_;
	int normalKSearch_;
	float maxGroundAngle_;
	float clusterRadius_;
	int minClusterSize_;
	bool flatObstaclesDetected_;
	float minGroundHeight_;
	float maxGroundHeight_;
	bool normalsSegmentation_;
	float noiseFilteringRadius_;
	int noiseFilteringMinNeighbors_;
	bool scan2dUnknownSpaceFilled_;
	bool rayTracing_;
	bool erode_; // false по умолчанию
	float footprintRadius_;
	float occupancyThr_;
	float probHit_;
	float probMiss_;
	float probClampingMin_;
	float probClampingMax_;
	float minSemanticRange_;
	float maxSemanticRange_;
	float minSemanticRangeSqr_;
	float maxSemanticRangeSqr_;
	int temporarilyOccupiedCellsColor_;
	bool showTemporarilyOccupiedCells_;
	int maxTemporaryLocalMaps_;

	std::map<int, LocalMap> localMaps_;
	std::map<int, Transform> poses_;

	int xMin_;
	int yMin_;
	cv::Mat map_;
	cv::Mat colors_;  // b, g, r

	std::unique_ptr<CachedMap> cachedMap_;

	std::list<std::pair<int, int>> temporarilyOccupiedCells_;

	std::list<LocalMap> temporaryLocalMaps_;
	std::list<Transform> temporaryPoses_;
};

}

#endif /* CORELIB_SRC_OCCUPANCYGRID_H_ */
