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

std::shared_ptr<ObstacleDilation::LocalMap> ObstacleDilation::dilate(
	const LocalMap& localMap) const
{
	UASSERT(dilationSize_ > 0);
	float minXf = std::numeric_limits<float>::max();
	float minYf = std::numeric_limits<float>::max();
	float maxXf = std::numeric_limits<float>::min();
	float maxYf = std::numeric_limits<float>::min();
	for (int i = 0; i < localMap.points.cols(); i += 2)
	{
		float x = localMap.points.coeff(0, i);
		float y = localMap.points.coeff(1, i);
		minXf = std::min(minXf, x);
		minYf = std::min(minYf, y);
		maxXf = std::max(maxXf, x);
		maxYf = std::max(maxYf, y);
	}

	int minY = std::floor(minYf / cellSize_);
	int minX = std::floor(minXf / cellSize_);
	int maxY = std::floor(maxYf / cellSize_);
	int maxX = std::floor(maxXf / cellSize_);
	minY -= dilationSize_;
	minX -= dilationSize_;
	maxY += dilationSize_;
	maxX += dilationSize_;

	cv::Mat grid = cv::Mat(maxY - minY + 1, maxX - minX + 1, CV_8U, 255);
	for (int i = 0; i < localMap.points.cols(); i += 2)
	{
		float yf = localMap.points.coeff(1, i);
		float xf = localMap.points.coeff(0, i);
		int y = std::floor(yf / cellSize_) - minY;
		int x = std::floor(xf / cellSize_) - minX;
		bool occupied = (i < localMap.numObstacles);
		if (occupied)
		{
			grid.at<std::uint8_t>(y, x) = 100;
		}
		else
		{
			grid.at<std::uint8_t>(y, x) = 0;
		}
	}

	cv::Mat dilated = semanticDilation_->dilate(grid, {(std::uint8_t)100},
		true /* inverseBackground */);
	
	std::vector<std::pair<int, int>> occupiedCells;
	std::vector<std::pair<int, int>> emptyCells;
	for (int y = 0; y < dilated.rows; y++)
	{
		for (int x = 0; x < dilated.cols; x++)
		{
			std::uint8_t value = dilated.at<std::uint8_t>(y, x);
			if (value == 100)
			{
				occupiedCells.emplace_back(y, x);
			}
			else if (value == 0)
			{
				emptyCells.emplace_back(y, x);
			}
		}
	}

	std::shared_ptr<LocalMap> dilatedLocalMap = std::make_shared<LocalMap>();
	dilatedLocalMap->numObstacles = occupiedCells.size() * 2;
	dilatedLocalMap->numEmpty = emptyCells.size() * 2;
	dilatedLocalMap->points.resize(3, dilatedLocalMap->numObstacles + dilatedLocalMap->numEmpty);
	dilatedLocalMap->colors.reserve(dilatedLocalMap->numObstacles + dilatedLocalMap->numEmpty);
	for (int i = 0, cellI = 0; i < dilatedLocalMap->numObstacles + dilatedLocalMap->numEmpty;
		i += 2, cellI++)
	{
		int x;
		int y;
		if (cellI < occupiedCells.size())
		{
			const std::pair<int, int>& occupiedCell = occupiedCells[cellI];
			x = occupiedCell.second;
			y = occupiedCell.first;
		}
		else
		{
			const std::pair<int, int>& emptyCell = emptyCells[cellI - occupiedCells.size()];
			x = emptyCell.second;
			y = emptyCell.first;
		}
		float xf = (x + minX + 0.5f) * cellSize_;
		float yf = (y + minY + 0.5f) * cellSize_;
		dilatedLocalMap->points(0, i) = xf;
		dilatedLocalMap->points(1, i) = yf;
		dilatedLocalMap->points(2, i) = 0.0f;
		dilatedLocalMap->colors.push_back(Color::missingColor);
		dilatedLocalMap->points(0, i + 1) = xf + cellSize_ * 0.5f;
		dilatedLocalMap->points(1, i + 1) = yf + cellSize_ * 0.5f;
		dilatedLocalMap->points(2, i + 1) = 0.0f;
		dilatedLocalMap->colors.push_back(Color::missingColor);
	}
	return dilatedLocalMap;
}

}
