#include <rtabmap/core/RayTracing.h>
#include <rtabmap/core/LocalMap.h>

#include <kas_utils/time_measurer.h>

namespace rtabmap {

RayTracing::RayTracing(const Parameters& parameters)
{
    parseParameters(parameters);
}

void RayTracing::parseParameters(const Parameters& parameters)
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

void RayTracing::traceRays(MultiArray<std::uint8_t, 2>& grid, const Cell& origin) const
{
    UASSERT(origin.inFrame(grid.shape()[0], grid.shape()[1]));
    for (const Ray& ray : rays_)
    {
        maybeEmptyCells_.clear();
        emptyCells_.clear();
        bool encounteredObstacle = false;
        int i = 0;
        for (Cell cell : ray.cells)
        {
            cell += origin;
            if (!cell.inFrame(grid.shape()[0], grid.shape()[1]))
            {
                break;
            }
            const std::uint8_t& cellValue = grid[{cell.y, cell.x}];
            if (cellValue == LocalMap2d::ColoredGrid::occupiedCellValue ||
                cellValue == LocalMap2d::ColoredGrid::ignoredOccupiedCellValue)
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
                grid[{cell.y, cell.x}] = LocalMap2d::ColoredGrid::maybeEmptyCellValue;
            }
            for (const Cell& cell : emptyCells_)
            {
                grid[{cell.y, cell.x}] = LocalMap2d::ColoredGrid::emptyCellValue;
            }
        }
    }
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
    std::list<Cell> circle = bresenhamCircle(0, 0, maxTracingRange_);
    double scale = 1.0 * maxVisibleRange_ / maxTracingRange_;
    for (const Cell& cell : circle)
    {
        Cell cellForRayTracing = cell * scale;
        std::list<Cell> line = bresenhamLine(Cell{0, 0}, cellForRayTracing);
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

}
