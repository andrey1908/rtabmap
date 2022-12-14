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
#include <rtabmap/core/LocalMapBuilder.h>
#include <rtabmap/core/ColoredOccupancyGrid.h>
#include <rtabmap/core/TemporaryColoredOccupancyGrid.h>
#include <rtabmap/core/ColoredOccupancyGridInterface.h>

#include <memory>
#include <optional>
#include <climits>

namespace rtabmap {

class OccupancyGridBuilder : public ColoredOccupancyGridInterface
{
public:
	OccupancyGridBuilder(const ParametersMap& parameters = ParametersMap());
	void parseParameters(const ParametersMap& parameters);

	LocalMap createLocalMap(const Signature& signature) const;

	void addLocalMap(int nodeId, LocalMap localMap);
	void addLocalMap(int nodeId, const Transform& pose, LocalMap localMap);
	void addTemporaryLocalMap(const Transform &pose, LocalMap localMap);

	void cacheCurrentMap();

	void updatePoses(const std::map<int, Transform>& updatedPoses,
			const std::list<Transform>& updatedTemporaryPoses,
			int lastNodeIdForCachedMap = -1);

	OccupancyGrid getOccupancyGrid() const;
	OccupancyGrid getProbOccupancyGrid() const;
	ColorGrid getColorGrid() const;

	float cellSize() const { return cellSize_; }
	std::pair<float, float> getGridOrigin() const;
	int maxTemporaryLocalMaps() const { return temporaryColoredOccupancyGrid_.maxTemporaryLocalMaps(); }
	const std::map<int, Node>& nodes() const { return coloredOccupancyGrid_.nodes(); }
	const cv::Mat& lastDilatedSemantic() const { return localMapBuilder_.lastDilatedSemantic(); }

	void resetAll();
	void resetTemporaryMap();

private:
	float cellSize_;
	float sensorBlindRange2d_;
	float sensorBlindRange2dSqr_;

	LocalMapBuilder localMapBuilder_;
	ColoredOccupancyGrid coloredOccupancyGrid_;
	TemporaryColoredOccupancyGrid temporaryColoredOccupancyGrid_;
};

}

#endif /* CORELIB_SRC_OCCUPANCYGRID_H_ */
