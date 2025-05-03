#include <rtabmap/core/RayTracing.h>
#include <rtabmap/core/LocalMap.h>

#include <kas_utils/time_measurer.h>
#include <kas_utils/utils.hpp>

namespace rtabmap {

using kas_utils::castArray;

template <int Dims>
RayTracing<Dims>::RayTracing(const Parameters& parameters)
{
    parseParameters(parameters);
}

template <int Dims>
void RayTracing<Dims>::parseParameters(const Parameters& parameters)
{
    cellSize_ = parameters.cellSize;
    maxVisibleRangeF_ = parameters.maxVisibleRange;
    maxTracingRangeF_ = parameters.maxTracingRange;
    sensorBlindRange2dF_ = parameters.sensorBlindRange2d;
    traceIntoUnknownSpace_ = parameters.traceIntoUnknownSpace;
    UASSERT(cellSize_ > 0.0f);
    UASSERT(maxVisibleRangeF_ >= 0.0f);
    UASSERT(maxTracingRangeF_ >= 0.0f);
    UASSERT(maxVisibleRangeF_ >= maxTracingRangeF_);

    maxVisibleRange_ = std::lround(maxVisibleRangeF_ / cellSize_);
    maxTracingRange_ = std::lround(maxTracingRangeF_ / cellSize_);
    maxTracingRangeSqr_ = maxTracingRange_ * maxTracingRange_;
    if (sensorBlindRange2dF_ >= 0.0f)
    {
        float sensorBlindRange2dInCells = sensorBlindRange2dF_ / cellSize_;
        sensorBlindRange2dSqr_ = std::ceil(sensorBlindRange2dInCells * sensorBlindRange2dInCells);
    }
    else
    {
        sensorBlindRange2dSqr_ = -1;
    }
    computeRays();
}

template <int Dims>
void RayTracing<Dims>::traceRays(MultiArray<std::uint8_t, Dims>& grid, const Cell& origin) const
{
    UASSERT(origin.inFrame(castArray<int>(grid.shape())));
    for (const Ray& ray : rays_)
    {
        maybeEmptyCells_.clear();
        emptyCells_.clear();
        bool encounteredObstacle = false;
        int i = 0;
        for (Cell cell : ray.cells)
        {
            cell += origin;
            if (!cell.inFrame(castArray<int>(grid.shape())))
            {
                break;
            }
            const std::uint8_t& cellValue = grid[castArray<std::size_t>(cell)];
            if (cellValue == LocalMap<Dims>::ColoredGrid::occupiedCellValue ||
                cellValue == LocalMap<Dims>::ColoredGrid::ignoredOccupiedCellValue)
            {
                encounteredObstacle = true;
                break;
            }
            if (i < ray.numMaybeEmpty)
            {
                maybeEmptyCells_.emplace_back(cell);
            }
            else if (i < ray.numEmpty)
            {
                emptyCells_.emplace_back(cell);
            }
            i++;
        }
        if (encounteredObstacle || traceIntoUnknownSpace_)
        {
            for (const Cell& cell : maybeEmptyCells_)
            {
                grid[castArray<std::size_t>(cell)] = LocalMap<Dims>::ColoredGrid::maybeEmptyCellValue;
            }
            for (const Cell& cell : emptyCells_)
            {
                grid[castArray<std::size_t>(cell)] = LocalMap<Dims>::ColoredGrid::emptyCellValue;
            }
        }
    }
}

template <int Dims>
std::list<typename RayTracing<Dims>::Cell2d> RayTracing<Dims>::bresenhamLine2d(const Cell2d& start, const Cell2d& end)
{
    std::list<Cell2d> line;
    int x0 = start[1];
    int y0 = start[0];
    int x1 = end[1];
    int y1 = end[0];

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
        line.emplace_back(Cell2d{ly, lx});
        if (D >= 0)
        {
            y++;
            D -= 2 * dx;
            if (x != dx)
            {
                // 4 connected line
                int lx = x0 + x * xx + y * yx;
                int ly = y0 + x * xy + y * yy;
                line.emplace_back(Cell2d{ly, lx});
            }
        }
        D += 2 * dy;
    }

    return line;
}

template <int Dims>
std::list<typename RayTracing<Dims>::Cell3d> RayTracing<Dims>::bresenhamLine3d(const Cell3d& start, const Cell3d& end)
{
    std::list<Cell3d> line;
    int x1 = start[2];
    int y1 = start[1];
    int z1 = start[0];
    int x2 = end[2];
    int y2 = end[1];
    int z2 = end[0];

    line.emplace_back(Cell3d{z1, y1, x1});
    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    int dz = std::abs(z2 - z1);
    int xs;
    int ys;
    int zs;
    if (x2 > x1)
        xs = 1;
    else
        xs = -1;
    if (y2 > y1)
        ys = 1;
    else
        ys = -1;
    if (z2 > z1)
        zs = 1;
    else
        zs = -1;

    // Driving axis is X-axis"
    if (dx >= dy && dx >= dz)
    {
        int p1 = 2 * dy - dx;
        int p2 = 2 * dz - dx;
        while (x1 != x2)
        {
            x1 += xs;
            if (p1 >= 0)
            {
                y1 += ys;
                p1 -= 2 * dx;
            }
            if (p2 >= 0)
            {
                z1 += zs;
                p2 -= 2 * dx;
            }
            p1 += 2 * dy;
            p2 += 2 * dz;
            line.emplace_back(Cell3d{z1, y1, x1});
        }
    }
    // Driving axis is Y-axis"
    else if (dy >= dx && dy >= dz)
    {
        int p1 = 2 * dx - dy;
        int p2 = 2 * dz - dy;
        while (y1 != y2) {
            y1 += ys;
            if (p1 >= 0)
            {
                x1 += xs;
                p1 -= 2 * dy;
            }
            if (p2 >= 0)
            {
                z1 += zs;
                p2 -= 2 * dy;
            }
            p1 += 2 * dx;
            p2 += 2 * dz;
            line.emplace_back(Cell3d{z1, y1, x1});
        }
    }
    // Driving axis is Z-axis"
    else
    {
        int p1 = 2 * dy - dz;
        int p2 = 2 * dx - dz;
        while (z1 != z2)
        {
            z1 += zs;
            if (p1 >= 0)
            {
                y1 += ys;
                p1 -= 2 * dz;
            }
            if (p2 >= 0)
            {
                x1 += xs;
                p2 -= 2 * dz;
            }
            p1 += 2 * dy;
            p2 += 2 * dx;
            line.emplace_back(Cell3d{z1, y1, x1});
        }
    }

    return line;
}

template <int Dims>
std::list<typename RayTracing<Dims>::Cell> RayTracing<Dims>::bresenhamLine(const Cell& start, const Cell& end)
{
    if constexpr(Dims == 2)
    {
        return bresenhamLine2d(start, end);
    }
    if constexpr(Dims == 3)
    {
        return bresenhamLine3d(start, end);
    }
}

template <int Dims>
void RayTracing<Dims>::addCirclePoints(std::set<Cell2d>& circle, int cy, int cx, int y, int x)
{
    circle.emplace(Cell2d{cy + y, cx + x});
    circle.emplace(Cell2d{cy + y, cx - x});
    circle.emplace(Cell2d{cy - y, cx + x});
    circle.emplace(Cell2d{cy - y, cx - x});
    circle.emplace(Cell2d{cy + x, cx + y});
    circle.emplace(Cell2d{cy + x, cx - y});
    circle.emplace(Cell2d{cy - x, cx + y});
    circle.emplace(Cell2d{cy - x, cx - y});
}

template <int Dims>
std::list<typename RayTracing<Dims>::Cell2d> RayTracing<Dims>::bresenhamCircle2d(const Cell2d& center, int r)
{
    std::set<Cell2d> circle;  // use std::set to remove duplicated cells
    int cy = center[0];
    int cx = center[1];
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
    return {circle.begin(), circle.end()};
}

template <int Dims>
std::list<typename RayTracing<Dims>::Cell3d> RayTracing<Dims>::bresenhamSphere3d(const Cell3d& center, int r)
{
    /// TODO: this function may skip some points at sphere's poles
    std::set<Cell3d> sphere;  // use std::set to remove duplicated cells
    int prevCircleR = r;
    int currentCircleR = r;
    for (int z = 0; z <= r; z++)
    {
        int nextZ = std::min(z + 1, r);
        int nextCircleR = std::round(std::sqrt(r * r - nextZ * nextZ));

        int maxCircleR = std::floor((prevCircleR + currentCircleR) / 2.0);
        int minCircleR = std::ceil((currentCircleR + nextCircleR) / 2.0);
        maxCircleR = std::max(maxCircleR, 1);
        minCircleR = std::max(minCircleR, 1);
        for (int circleR = minCircleR; circleR <= maxCircleR; circleR++)
        {
            std::list<Cell2d> circle = bresenhamCircle2d({center[1], center[2]}, circleR);
            for (const Cell2d& cell : circle)
            {
                sphere.emplace(Cell3d{z, cell[0], cell[1]});
                sphere.emplace(Cell3d{-z, cell[0], cell[1]});
            }
        }

        prevCircleR = currentCircleR;
        currentCircleR = nextCircleR;
    }
    return {sphere.begin(), sphere.end()};
}

template <int Dims>
std::list<typename RayTracing<Dims>::Cell> RayTracing<Dims>::getRaysEndCells(const Cell& center, int r)
{
    if constexpr(Dims == 2)
    {
        return bresenhamCircle2d(center, r);
    }
    if constexpr(Dims == 3)
    {
        return bresenhamSphere3d(center, r);
    }
}

template <int Dims>
void RayTracing<Dims>::computeRays()
{
    rays_.clear();
    std::list<Cell> raysEndCells = getRaysEndCells({}, maxTracingRange_);
    double scale = 1.0 * maxVisibleRange_ / maxTracingRange_;
    for (Cell rayEndCell : raysEndCells)
    {
        rayEndCell = rayEndCell * scale;
        std::list<Cell> line = bresenhamLine({}, rayEndCell);
        rays_.emplace_back();
        Ray& ray = rays_.back();
        int numMaybeEmpty = -1;
        int numEmpty = -1;
        int i = 0;
        for (const Cell& cell : line)
        {
            if (cell.rangeSqr() > sensorBlindRange2dSqr_ && numMaybeEmpty == -1)
            {
                numMaybeEmpty = i;
            }
            if (cell.rangeSqr() > maxTracingRangeSqr_ && numEmpty == -1)
            {
                numEmpty = i;
            }

            ray.cells.emplace_back(cell);
            i++;
        }
        if (numMaybeEmpty == -1)
        {
            numMaybeEmpty = line.size();
        }
        if (numEmpty == -1)
        {
            numEmpty = line.size();
        }
        numMaybeEmpty = std::min(numMaybeEmpty, numEmpty);

        ray.numMaybeEmpty = numMaybeEmpty;
        ray.numEmpty = numEmpty;
    }

    int maxNumMaybeEmpty = 0;
    int maxNumEmpty = 0;
    for (const Ray& ray : rays_)
    {
        maxNumMaybeEmpty = std::max(maxNumMaybeEmpty, ray.numMaybeEmpty);
        maxNumEmpty = std::max(maxNumEmpty, ray.numEmpty);
    }
    maybeEmptyCells_.reserve(maxNumMaybeEmpty);
    emptyCells_.reserve(maxNumEmpty);
}

template class RayTracing<2>;
template class RayTracing<3>;

}
