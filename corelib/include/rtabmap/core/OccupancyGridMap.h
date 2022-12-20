#pragma once

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/LocalMapBuilder.h>
#include <rtabmap/core/OccupancyGridBuilder.h>
#include <rtabmap/core/TemporaryOccupancyGridBuilder.h>
#include <rtabmap/core/ObstacleDilation.h>
#include <rtabmap/core/BaseClasses.h>

#include <memory>
#include <optional>
#include <climits>

namespace rtabmap {

class OccupancyGridMap : public BaseClasses
{
public:
	OccupancyGridMap(const ParametersMap& parameters = ParametersMap());
	void parseParameters(const ParametersMap& parameters);

	std::shared_ptr<const LocalMap> createLocalMap(const Signature& signature) const;

	void addLocalMap(int nodeId, std::shared_ptr<const LocalMap> localMap);
	void addLocalMap(int nodeId, std::shared_ptr<const LocalMap> localMap,
		const Transform& pose);
	void addTemporaryLocalMap(std::shared_ptr<const LocalMap> localMap,
		const Transform &pose);

	void cacheCurrentMap();

	void updatePoses(const std::map<int, Transform>& updatedPoses,
			const std::list<Transform>& updatedTemporaryPoses,
			int lastNodeIdForCachedMap = -1);

	OccupancyGrid getOccupancyGrid() const;
	OccupancyGrid getProbOccupancyGrid() const;
	ColorGrid getColorGrid() const;

	float cellSize() const { return cellSize_; }
	std::pair<float, float> getGridOrigin() const;
	int maxTemporaryLocalMaps() const
		{ return temporaryOccupancyGridBuilder_->maxTemporaryLocalMaps(); }
	const std::map<int, Node>& nodes() const { return occupancyGridBuilder_->nodes(); }
	const std::map<int, const std::shared_ptr<const LocalMap>>&
		localMapsWithoutObstacleDilation() const { return localMapsWithoutObstacleDilation_; }
	const cv::Mat& lastDilatedSemantic() const
		{ return localMapBuilder_->lastDilatedSemantic(); }

	void resetAll();
	void resetTemporaryMap();

private:
	float cellSize_;

	std::unique_ptr<LocalMapBuilder> localMapBuilder_;
	std::unique_ptr<ObstacleDilation> obstacleDilation_;
	std::unique_ptr<OccupancyGridBuilder> occupancyGridBuilder_;
	std::unique_ptr<TemporaryOccupancyGridBuilder> temporaryOccupancyGridBuilder_;

	std::map<int, const std::shared_ptr<const LocalMap>> localMapsWithoutObstacleDilation_;
};

}
