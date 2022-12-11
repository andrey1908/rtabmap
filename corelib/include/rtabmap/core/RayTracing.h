#pragma once

#include <rtabmap/core/Parameters.h>

#include <opencv2/opencv.hpp>
#include <vector>
#include <list>

namespace rtabmap {

class RayTracing
{
public:
	static constexpr std::int8_t occupiedCellValue = 100;
	static constexpr std::int8_t emptyCellValue = 0;

public:
	struct Cell
	{
		Cell operator*(const int i) const
		{
			Cell cell;
			cell.y = y * i;
			cell.x = x * i;
			return cell;
		}
		Cell operator*(const double d) const
		{
			Cell cell;
			cell.y = std::lround(y * d);
			cell.x = std::lround(x * d);
			return cell;
		}
		Cell& operator+=(const Cell& other)
		{
			y += other.y;
			x += other.x;
			return *this;
		}
		inline int rangeSqr() const
		{
			return y * y + x * x;
		}
		inline bool inFrame(int h, int w) const
		{
			return y >= 0 && x >= 0 && y < h && x < w;
		}
		int y;
		int x;
	};

private:
	struct Ray
	{
		std::vector<Cell> cells;
		int lightRayLength;
	};

public:
	RayTracing(const ParametersMap& parameters = ParametersMap());
	void parseParameters(const ParametersMap& parameters);
	~RayTracing() {};

	void traceRays(cv::Mat& grid, const Cell& origin) const;

private:
	void addCirclePoints(std::list<Cell>& circle, int cy, int cx, int y, int x);
	std::list<Cell> bresenhamCircle(int cy, int cx, int r);
	std::list<Cell> bresenhamLine(const Cell& start, const Cell& end);
	void computeRays();

private:
	bool initializeRayTracing_;
	float cellSize_;
	float maxVisibleRangeF_;
	float maxRayTracingRangeF_;

	int maxVisibleRange_;
	int maxRayTracingRange_;
	int maxRayTracingRangeSqr_;
	bool traceRaysIntoUnknownSpace_;

	std::vector<Ray> rays_;

	// buffer for ray tracing
	mutable std::vector<Cell> litCells_;
};

}
