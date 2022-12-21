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
private:
	using CounterType = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
	using ColorsType = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

public:
	TemporaryOccupancyGridBuilder(const ParametersMap& parameters = ParametersMap());
	void parseParameters(const ParametersMap& parameters);

	void addLocalMap(std::shared_ptr<const LocalMap> localMap, const Transform& pose);

	void updatePoses(const std::list<Transform>& updatedPoses);

	OccupancyGrid getOccupancyGrid() const;
	OccupancyGrid getOccupancyGrid(const MapLimits& roi) const;
	OccupancyGrid getProbOccupancyGrid() const;
	OccupancyGrid getProbOccupancyGrid(const MapLimits& roi) const;
	ColorGrid getColorGrid() const;
	ColorGrid getColorGrid(const MapLimits& roi) const;

	int maxTemporaryLocalMaps() const { return maxTemporaryLocalMaps_; }
	const MapLimits& mapLimits() const { return mapLimits_; }

	void reset();

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
	float temporaryHitProb_;
	float temporaryOccupancyProbThr_;
	int updated_;
	int maxTemporaryLocalMaps_;

	std::list<Node> nodes_;
	MapLimits mapLimits_;
	CounterType hitCounter_;
	CounterType missCounter_;
	ColorsType colors_;

	Eigen::Matrix<std::int8_t, Eigen::Dynamic, Eigen::Dynamic> probabilities_;
	Eigen::Matrix<std::int8_t, Eigen::Dynamic, Eigen::Dynamic> probabilitiesThr_;
};

}
