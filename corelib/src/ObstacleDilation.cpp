#include <rtabmap/core/ObstacleDilation.h>
#include <rtabmap/utilite/ULogger.h>

#include <limits>

#include "time_measurer/time_measurer.h"

namespace rtabmap {

ObstacleDilation::ObstacleDilation(const ParametersMap& parameters) :
	cellSize_(Parameters::defaultGridCellSize()),
	dilationSizeMeters_(Parameters::defaultObstacleDilationDilationSize())
{
	parseParameters(parameters);
}

void ObstacleDilation::parseParameters(const ParametersMap& parameters)
{
	Parameters::parse(parameters, Parameters::kGridCellSize(), cellSize_);
	Parameters::parse(parameters, Parameters::kObstacleDilationDilationSize(),
		dilationSizeMeters_);
	UASSERT(dilationSizeMeters_ >= 0.0f);

	dilationSize_ = std::ceil(dilationSizeMeters_ / cellSize_);

	semanticDilation_ = std::make_unique<SemanticDilation>(dilationSize_);
}

std::shared_ptr<LocalMap> ObstacleDilation::dilate(
	const LocalMap& localMap) const
{
	UASSERT(dilationSize_ > 0);
	LocalMap::ColoredGrid coloredGrid = localMap.toColoredGrid();
	coloredGrid.grid = semanticDilation_->dilate(coloredGrid.grid,
		{LocalMap::ColoredGrid::occupiedCellValue} /* backgroundColors */,
		true /* inverseBackground */);

	auto dilatedLocalMap = std::make_shared<LocalMap>(
		coloredGrid, 0.0f, localMap.pointsDuplicated());
	return dilatedLocalMap;
}

}
