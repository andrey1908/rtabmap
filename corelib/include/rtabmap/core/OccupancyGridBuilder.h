#pragma once

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/LocalMapBuilder.h>
#include <rtabmap/core/BaseClasses.h>

#include <list>
#include <map>
#include <utility>
#include <optional>
#include <Eigen/Core>

namespace rtabmap {

class OccupancyGridBuilder : BaseClasses
{
public:
	OccupancyGridBuilder(const ParametersMap& parameters = ParametersMap());
	void parseParameters(const ParametersMap& parameters);

	void addLocalMap(int nodeId, LocalMap localMap);
	void addLocalMap(int nodeId, LocalMap localMap, const Transform& pose);

	void cacheCurrentMap();
	void updatePoses(const std::map<int, Transform>& updatedPoses,
		int lastNodeIdForCachedMap = -1);

	OccupancyGrid getOccupancyGrid() const;
	OccupancyGrid getProbOccupancyGrid() const;
	ColorGrid getColorGrid() const;

	OccupancyGrid getOccupancyGrid(const MapLimits& roi) const;
	OccupancyGrid getProbOccupancyGrid(const MapLimits& roi) const;
	ColorGrid getColorGrid(const MapLimits& roi) const;

	const std::map<int, Node>& nodes() const { return nodes_; }
	const MapLimits& mapLimits() const { return mapLimits_; }

private:
	bool checkIfCachedMapCanBeUsed(const std::map<int, Transform>& updatedPoses);
	void useCachedMap();
	int tryToUseCachedMap(const std::map<int, Transform>& updatedPoses);

	TransformedLocalMap transformLocalMap(const LocalMap& localMap, const Transform& transform);
	void createOrResizeMap(const MapLimits& newMapLimits);
	void deployLocalMap(int nodeId);
	void deployLocalMap(const Node& node);

	void clear();

private:
	float cellSize_;
	float missProb_;
	float miss_;
	float hitProb_;
	float hit_;
	float minClampingProb_;
	float minClamping_;
	float maxClampingProb_;
	float maxClamping_;
	float occupancyProbThr_;
	float occupancyThr_;
	float unknown_;
	int temporarilyOccupiedCellColorRgb_;
	Color temporarilyOccupiedCellColor_;
	bool showTemporarilyOccupiedCells_;

	std::map<int, Node> nodes_;
	MapLimits mapLimits_;
	Eigen::MatrixXf map_;
	Eigen::MatrixXi colors_;
	std::list<std::pair<int, int>> temporarilyOccupiedCells_;

	std::map<int, Transform> cachedPoses_;
	MapLimits cachedMapLimits_;
	Eigen::MatrixXf cachedMap_;
	Eigen::MatrixXi cachedColors_;
	std::list<std::pair<int, int>> cachedTemporarilyOccupiedCells_;
};

}
