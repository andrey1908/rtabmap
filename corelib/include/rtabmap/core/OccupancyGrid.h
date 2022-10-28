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
#include <optional>
#include <climits>

namespace rtabmap {

class RTABMAP_EXP OccupancyGrid
{
public:
	using OccupancyMap = Eigen::Matrix<char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
	using ColorsMap = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

private:
	inline static float logodds(double probability)
	{
		return (float) log(probability/(1-probability));
	}

	inline static double probability(double logodds)
	{
		return 1. - ( 1. / (1. + exp(logodds)));
	}

	struct MapLimits
	{
		MapLimits() :
			minX(std::numeric_limits<int>::max()),
			minY(std::numeric_limits<int>::max()),
			maxX(std::numeric_limits<int>::min() + 1),
			maxY(std::numeric_limits<int>::min() + 1) {}
		bool operator==(const MapLimits& other) const
		{
			return minX == other.minX && minY == other.minY && maxX == other.maxX && maxY == other.maxY;
		}
		bool operator!=(const MapLimits& other) const
		{
			return !(*this == other);
		}
		bool valid() const
		{
			return minX != std::numeric_limits<int>::max();
		}
		void update(int x, int y)
		{
			if (x < minX)
				minX = x;
			if (x > maxX - 1)
				maxX = x + 1;
			if (y < minY)
				minY = y;
			if (y > maxY - 1)
				maxY = y + 1;
		}
		int width() const
		{
			return maxX - minX;
		}
		int height() const
		{
			return maxY - minY;
		}
		static MapLimits unite(const MapLimits& a, const MapLimits& b)
		{
			MapLimits res;
			res.minX = std::min(a.minX, b.minX);
			res.minY = std::min(a.minY, b.minY);
			res.maxX = std::max(a.maxX, b.maxX);
			res.maxY = std::max(a.maxY, b.maxY);
			return res;
		}
		static MapLimits intersect(const MapLimits& a, const MapLimits& b)
		{
			MapLimits res;
			res.minX = std::max(a.minX, b.minX);
			res.minY = std::max(a.minY, b.minY);
			res.maxX = std::min(a.maxX, b.maxX);
			res.maxY = std::min(a.maxY, b.maxY);
			if (res.minX > res.maxX)
				res.minX = res.maxX;
			if (res.minY > res.maxY)
				res.minY = res.maxY;
			return res;
		}
		int minX;
		int minY;
		int maxX;
		int maxY;
	};

public:
	struct LocalMap
	{
		int numGround, numEmpty, numObstacles;
		Eigen::Matrix3Xf points;
		std::vector<int> colors;
	};

	struct Node
	{
		template <typename LocalPoseType, typename LocalMapType>
		Node(LocalPoseType&& otherLocalPose, LocalMapType&& otherLocalMap) :
			localPose(std::forward<LocalPoseType>(otherLocalPose)),
			localMap(std::forward<LocalMapType>(otherLocalMap)),
			transformedLocalPoints2d(),
			localMapLimits() {}
		std::optional<Transform> localPose;
		const LocalMap localMap;
		std::optional<Eigen::Matrix2Xi> transformedLocalPoints2d;
		std::optional<MapLimits> localMapLimits;
	};

private:
	struct ColoredOccupancyMap
	{
		std::map<int, Node> nodes;
		MapLimits mapLimits;
		Eigen::MatrixXf map;
		Eigen::MatrixXi colors;
		std::list<std::pair<int, int>> temporarilyOccupiedCells;
	};

	struct TemporaryColoredOccupancyMap
	{
		std::map<int, Node> nodes;
		MapLimits mapLimits;
		Eigen::MatrixXi missCounter;
		Eigen::MatrixXi hitCounter;
		Eigen::MatrixXi colors;
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

	OccupancyMap getOccupancyMap(float & minX, float & minY) const;
	OccupancyMap getProbOccupancyMap(float & minX, float & minY) const;
	ColorsMap getColorsMap(float & minX, float & minY) const;

	float cellSize() const;
	int maxTemporaryLocalMaps() const;
	const std::map<int, Node> & nodes() const;
	const cv::Mat & lastDilatedSemantic() const;

	void resetAll();
	void resetTemporaryMap();

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
	cv::Mat dilate(const cv::Mat& rgb) const;
	LaserScan addSemanticToLaserScan(const LaserScan& scan, const cv::Mat& rgb,
			const std::vector<CameraModel>& cameraModels) const;
	LocalMap cvMatsToLocalMap(
			const cv::Mat & groundCells,
			const cv::Mat & emptyCells,
			const cv::Mat & obstacleCells) const;

	int tryToUseCachedMap(const std::map<int, Transform> & updatedPoses);

	void transformLocalMap(Node & node);
	void createOrResizeMap(ColoredOccupancyMap & map, const MapLimits & newMapLimits);
	void createOrResizeMap(TemporaryColoredOccupancyMap & map, const MapLimits & newMapLimits);
	void deployLocalMap(ColoredOccupancyMap & map, int nodeId);
	void deployLocalMap(TemporaryColoredOccupancyMap & map, int nodeId);
	void removeLocalMap(TemporaryColoredOccupancyMap & map, int nodeId);
	void clearColoredOccupancyMap(ColoredOccupancyMap& map);
	void clearColoredOccupancyMap(TemporaryColoredOccupancyMap& map);

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
	float footprintRadius_;
	float occupancyThr_;
	float probMiss_;
	float probHit_;
	float probClampingMin_;
	float probClampingMax_;
	float temporaryOccupancyThr_;
	float temporaryProbMiss_;
	float temporaryProbHit_;
	int semanticDilation_;
	float minSemanticRange_;
	float maxSemanticRange_;
	float minSemanticRangeSqr_;
	float maxSemanticRangeSqr_;
	int temporarilyOccupiedCellColor_;
	bool showTemporarilyOccupiedCells_;
	int maxTemporaryLocalMaps_;

	ColoredOccupancyMap map_;
	TemporaryColoredOccupancyMap temporaryMap_;
	std::unique_ptr<ColoredOccupancyMap> cachedMap_;

	mutable cv::Mat lastDilatedSemantic_;
};

}

#endif /* CORELIB_SRC_OCCUPANCYGRID_H_ */
