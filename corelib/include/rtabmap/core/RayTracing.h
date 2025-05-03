#pragma once

#include <rtabmap/utilite/ULogger.h>

#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>
#include <vector>
#include <array>
#include <list>

#include <cstdint>

#include <kas_utils/multi_array.hpp>

namespace rtabmap {

using kas_utils::MultiArray;

template <int Dims>
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

    template <int CellDims>
    struct CellImpl : public std::array<int, CellDims>
    {
        CellImpl operator*(const int i) const
        {
            CellImpl cell;
            for (int dim = 0; dim < CellDims; dim++)
            {
                cell[dim] = this->operator[](dim) * i;
            }
            return cell;
        }
        CellImpl operator*(const double d) const
        {
            CellImpl cell;
            for (int dim = 0; dim < CellDims; dim++)
            {
                cell[dim] = std::lround(this->operator[](dim) * d);
            }
            return cell;
        }
        CellImpl& operator+=(const CellImpl& other)
        {
            for (int dim = 0; dim < CellDims; dim++)
            {
                this->operator[](dim) += other[dim];
            }
            return *this;
        }
        int rangeSqr() const
        {
            int rangeSqr = 0;
            for (int dim = 0; dim < CellDims; dim++)
            {
                int axisRange = this->operator[](dim);
                rangeSqr += axisRange * axisRange;
            }
            return rangeSqr;
        }
        bool inFrame(const std::array<int, CellDims>& shape) const
        {
            for (int dim = 0; dim < CellDims; dim++)
            {
                int coord = this->operator[](dim);
                if (coord < 0 || coord >= shape[dim])
                {
                    return false;
                }
            }
            return true;
        }
    };

    typedef CellImpl<2> Cell2d;
    typedef CellImpl<3> Cell3d;
    typedef CellImpl<Dims> Cell;

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

    void traceRays(MultiArray<std::uint8_t, Dims>& grid, const Cell& origin) const;

    float maxTracingRange() const
    {
        return maxTracingRangeF_;
    }
    bool traceIntoUnknownSpace() const
    {
        return traceIntoUnknownSpace_;
    }

private:
    std::list<Cell2d> bresenhamLine2d(const Cell2d& start, const Cell2d& end);
    std::list<Cell3d> bresenhamLine3d(const Cell3d& start, const Cell3d& end);
    std::list<Cell> bresenhamLine(const Cell& start, const Cell& end);

    void addCirclePoints(std::set<Cell2d>& circle, int cy, int cx, int y, int x);
    std::list<Cell2d> bresenhamCircle2d(const Cell2d& center, int r);
    std::list<Cell3d> bresenhamSphere3d(const Cell3d& center, int r);
    std::list<Cell> getRaysEndCells(const Cell& center, int r);

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

typedef RayTracing<2> RayTracing2d;
typedef RayTracing<3> RayTracing3d;

}
