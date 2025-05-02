#pragma once

#include <rtabmap/utilite/ULogger.h>

#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>
#include <vector>
#include <list>

#include <cstdint>

#include <kas_utils/multi_array.hpp>

namespace rtabmap {

using kas_utils::MultiArray;

class RayTracing
{
public:
    struct Parameters
    {
        float cellSize = 0.1f;
        float maxVisibleRange = 100.0f;
        float maxTracingRange = 10.0f;
        float sensorBlindRange2d = -1.0f;  // disabled
        bool traceIntoUnknownSpace = false;

        static Parameters createParameters(const YAML::Node& node)
        {
            UASSERT(node.IsMap());
            Parameters parameters;
            if (node["CellSize"])
            {
                parameters.cellSize = node["CellSize"].as<float>();
            }
            if (node["MaxVisibleRange"])
            {
                parameters.maxVisibleRange = node["MaxVisibleRange"].as<float>();
            }
            if (node["MaxTracingRange"])
            {
                parameters.maxTracingRange = node["MaxTracingRange"].as<float>();
            }
            if (node["SensorBlindRange2d"])
            {
                parameters.sensorBlindRange2d = node["SensorBlindRange2d"].as<float>();
            }
            if (node["TraceIntoUnknownSpace"])
            {
                parameters.traceIntoUnknownSpace =
                    node["TraceIntoUnknownSpace"].as<bool>();
            }
            return parameters;
        }
    };

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
        int numMaybeEmpty;
        int numEmpty;
    };

public:
    RayTracing(const Parameters& parameters);
    void parseParameters(const Parameters& parameters);

    void traceRays(MultiArray<std::uint8_t, 2>& grid, const Cell& origin) const;

    float maxTracingRange() const
    {
        return maxTracingRangeF_;
    }
    bool traceIntoUnknownSpace() const
    {
        return traceIntoUnknownSpace_;
    }

private:
    void addCirclePoints(std::list<Cell>& circle, int cy, int cx, int y, int x);
    std::list<Cell> bresenhamCircle(int cy, int cx, int r);
    std::list<Cell> bresenhamLine(const Cell& start, const Cell& end);
    void computeRays();

private:
    float cellSize_;
    float maxVisibleRangeF_;
    int maxVisibleRange_;
    float maxTracingRangeF_;
    int maxTracingRange_;
    int maxTracingRangeSqr_;
    float sensorBlindRange2dF_;
    int sensorBlindRange2dSqr_;
    bool traceIntoUnknownSpace_;

    std::vector<Ray> rays_;

    // buffers for ray tracing
    mutable std::vector<Cell> maybeEmptyCells_;
    mutable std::vector<Cell> emptyCells_;
};

}
