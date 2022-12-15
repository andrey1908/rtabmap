#pragma once

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/LocalMapBuilder.h>
#include <rtabmap/core/OccupancyGridBuilder.h>
#include <rtabmap/core/TemporaryOccupancyGridBuilder.h>
#include <rtabmap/core/OccupancyGridClasses.h>

#include <memory>
#include <optional>
#include <climits>

namespace rtabmap {

class OccupancyGridMap : public OccupancyGridClasses
{
public:
	OccupancyGridMap(const ParametersMap& parameters = ParametersMap());
	void parseParameters(const ParametersMap& parameters);

	LocalMap createLocalMap(const Signature& signature) const;

	void addLocalMap(int nodeId, LocalMap localMap);
	void addLocalMap(int nodeId, LocalMap localMap, const Transform& pose);
	void addTemporaryLocalMap(LocalMap localMap, const Transform &pose);

	void cacheCurrentMap();

	void updatePoses(const std::map<int, Transform>& updatedPoses,
			const std::list<Transform>& updatedTemporaryPoses,
			int lastNodeIdForCachedMap = -1);

	OccupancyGrid getOccupancyGrid() const;
	OccupancyGrid getProbOccupancyGrid() const;
	ColorGrid getColorGrid() const;

	float cellSize() const { return cellSize_; }
	std::pair<float, float> getGridOrigin() const;
	int maxTemporaryLocalMaps() const { return temporaryOccupancyGridBuilder_.maxTemporaryLocalMaps(); }
	const std::map<int, Node>& nodes() const { return occupancyGridBuilder_.nodes(); }
	const cv::Mat& lastDilatedSemantic() const { return localMapBuilder_.lastDilatedSemantic(); }

	void resetAll();
	void resetTemporaryMap();

private:
	float cellSize_;
	float sensorBlindRange2d_;
	float sensorBlindRange2dSqr_;

	LocalMapBuilder localMapBuilder_;
	OccupancyGridBuilder occupancyGridBuilder_;
	TemporaryOccupancyGridBuilder temporaryOccupancyGridBuilder_;
};

}
