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

class TemporaryOccupancyGridBuilder : BaseClasses
{
public:
	TemporaryOccupancyGridBuilder(const ParametersMap& parameters = ParametersMap());
	void parseParameters(const ParametersMap& parameters);

	void addLocalMap(LocalMap localMap, const Transform& pose);

	void updatePoses(const std::list<Transform>& updatedPoses);

	OccupancyGrid getOccupancyGrid() const;
	OccupancyGrid getProbOccupancyGrid() const;
	ColorGrid getColorGrid() const;

	OccupancyGrid getOccupancyGrid(const MapLimits& roi) const;
	OccupancyGrid getProbOccupancyGrid(const MapLimits& roi) const;
	ColorGrid getColorGrid(const MapLimits& roi) const;

	int maxTemporaryLocalMaps() const { return maxTemporaryLocalMaps_; }
	const MapLimits& mapLimits() const { return mapLimits_; }

private:
	bool checkIfCachedMapCanBeUsed(const std::map<int, Transform>& updatedPoses);
	void useCachedMap();
	int tryToUseCachedMap(const std::map<int, Transform>& updatedPoses);

	TransformedLocalMap transformLocalMap(const LocalMap& localMap, const Transform& transform);
	void createOrResizeMap(const MapLimits& newMapLimits);
	void deployLastLocalMap();
	void deployLocalMap(const Node& node);
	void removeFirstLocalMap();
	void removeLocalMap(std::list<Node>::iterator nodeIt);

	void clear();

private:
	float cellSize_;
	float temporaryMissProb_;
	float miss_;
	float temporaryHitProb_;
	float hit_;
	float temporaryOccupancyProbThr_;
	float occupancyThr_;
	int maxTemporaryLocalMaps_;

	std::list<Node> nodes_;
	MapLimits mapLimits_;
	Eigen::MatrixXi missCounter_;
	Eigen::MatrixXi hitCounter_;
	Eigen::MatrixXi colors_;
};

}
