#include <rtabmap/core/RayTracing.h>
#include <rtabmap/utilite/ULogger.h>
#include "time_measurer/time_measurer.h"

namespace rtabmap {

RayTracing::RayTracing(const ParametersMap& parameters) :
	initializeRayTracing_(Parameters::defaultGridRayTracing()),
	cellSize_(Parameters::defaultGridCellSize()),
	maxVisibleRangeF_(Parameters::defaultRayTracingMaxVisibleRange()),
	maxRayTracingRangeF_(Parameters::defaultRayTracingMaxRayTracingRange()),
	traceRaysIntoUnknownSpace_(Parameters::defaultRayTracingTraceRaysIntoUnknownSpace())
{
	parseParameters(parameters);
	if (initializeRayTracing_ == false)
	{
		return;
	}
	maxVisibleRange_ = std::lround(maxVisibleRangeF_ / cellSize_);
	maxRayTracingRange_ = std::lround(maxRayTracingRangeF_ / cellSize_);
	maxRayTracingRangeSqr_ = maxRayTracingRange_ * maxRayTracingRange_;
	computeRays();
}

void RayTracing::parseParameters(const ParametersMap& parameters)
{
	Parameters::parse(parameters, Parameters::kGridRayTracing(), initializeRayTracing_);
	Parameters::parse(parameters, Parameters::kGridCellSize(), cellSize_);
	Parameters::parse(parameters, Parameters::kRayTracingMaxVisibleRange(), maxVisibleRangeF_);
	Parameters::parse(parameters, Parameters::kRayTracingMaxRayTracingRange(), maxRayTracingRangeF_);
	Parameters::parse(parameters, Parameters::kRayTracingTraceRaysIntoUnknownSpace(), traceRaysIntoUnknownSpace_);
	UASSERT(maxVisibleRangeF_ >= maxRayTracingRangeF_);
}

void RayTracing::addCirclePoints(std::list<Cell>& circle, int cy, int cx, int y, int x)
{
	circle.emplace_back(Cell{cy + y, cx + x});
	circle.emplace_back(Cell{cy + y, cx - x});
	circle.emplace_back(Cell{cy - y, cx + x});
	circle.emplace_back(Cell{cy - y, cx - x});
	circle.emplace_back(Cell{cy + x, cx + y});
	circle.emplace_back(Cell{cy + x, cx - y});
	circle.emplace_back(Cell{cy - x, cx + y});
	circle.emplace_back(Cell{cy - x, cx - y});
}

std::list<RayTracing::Cell> RayTracing::bresenhamCircle(int cy, int cx, int r)
{
	std::list<Cell> circle;
	int y = r;
	int x = 0;
	int d = 3 - 2 * r;
	addCirclePoints(circle, cy, cx, y, x);
	while (y >= x)
	{
		x++;
		if (d > 0)
		{
			y--;
			d = d + 4 * (x - y) + 10;
		}
		else
		{
			d = d + 4 * x + 6;
		}
		addCirclePoints(circle, cy, cx, y, x);
	}
	return circle;
}

std::list<RayTracing::Cell> RayTracing::bresenhamLine(const Cell& start, const Cell& end)
{
	std::list<Cell> line;
	int x0 = start.x;
	int y0 = start.y;
	int x1 = end.x;
	int y1 = end.y;

	int dx = x1 - x0;
	int dy = y1 - y0;
	int xsign = dx > 0 ? 1 : -1;
	int ysign = dy > 0 ? 1 : -1;
	dx = dx * xsign;
	dy = dy * ysign;

	int xx, xy, yx, yy;
	if (dx > dy)
	{
		xx = xsign;
		xy = 0;
		yx = 0;
		yy = ysign;
	}
	else
	{
		std::swap(dx, dy);
		xx = 0;
		xy = ysign;
		yx = xsign;
		yy = 0;
	}

	int D = 2 * dy - dx;
	int y = 0;
	for (int x = 0; x < dx + 1; x++)
	{
		int lx = x0 + x * xx + y * yx;
		int ly = y0 + x * xy + y * yy;
		line.emplace_back(Cell{ly, lx});
		if (D >= 0)
		{
			y++;
			D -= 2 * dx;
			if (x != dx)
			{
				// 4 connected line
				int lx = x0 + x * xx + y * yx;
				int ly = y0 + x * xy + y * yy;
				line.emplace_back(Cell{ly, lx});
			}
		}
		D += 2 * dy;
	}
	return line;
}

void RayTracing::computeRays()
{
	rays_.clear();
	std::list<Cell> circle = bresenhamCircle(0, 0, maxRayTracingRange_);
	double scale = 1.0 * maxVisibleRange_ / maxRayTracingRange_;
	for (const Cell& cell : circle)
	{
		Cell cellForRayTracing = cell * scale;
		std::list<Cell> line = bresenhamLine(Cell{0, 0}, cellForRayTracing);
		rays_.emplace_back();
		int lightRayLength = -1;
		int i = 0;
		for (const Cell& cell : line)
		{
			rays_.back().cells.emplace_back(cell);
			if (cell.rangeSqr() > maxRayTracingRangeSqr_ && lightRayLength == -1)
			{
				lightRayLength = i;
			}
			i++;
		}
		if (lightRayLength == -1)
		{
			lightRayLength = rays_.back().cells.size();
		}
		rays_.back().lightRayLength = lightRayLength;
	}

	int longestLightRay = 0;
	for (const Ray& ray : rays_)
	{
		longestLightRay = std::max(longestLightRay, ray.lightRayLength);
	}
	litCells_.reserve(longestLightRay);
}

void RayTracing::traceRays(cv::Mat& grid, const Cell& origin) const
{
	MEASURE_BLOCK_TIME(traceRays);
	UASSERT(initializeRayTracing_ == true);
	UASSERT(grid.type() == CV_8SC1);
	UASSERT(origin.inFrame(grid.rows, grid.cols));
	for (const Ray& ray : rays_)
	{
		litCells_.clear();
		bool encounteredObstacle = false;
		int i = 0;
		for (Cell cell : ray.cells)
		{
			cell += origin;
			if (!cell.inFrame(grid.rows, grid.cols))
			{
				break;
			}
			if (grid.at<std::int8_t>(cell.y, cell.x) == occupiedCellValue)
			{
				encounteredObstacle = true;
				break;
			}
			if (i < ray.lightRayLength)
			{
				litCells_.emplace_back(cell);
			}
			i++;
		}
		if (encounteredObstacle || traceRaysIntoUnknownSpace_)
		{
			for (const Cell& litCell : litCells_)
			{
				grid.at<std::int8_t>(litCell.y, litCell.x) = emptyCellValue;
			}
		}
	}
}

}
