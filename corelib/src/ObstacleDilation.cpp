#include <rtabmap/core/ObstacleDilation.h>
#include <limits>

#include <kas_utils/time_measurer.h>
#include <kas_utils/multi_array.hpp>

namespace rtabmap {

using kas_utils::MultiArray;

ObstacleDilation::ObstacleDilation(const Parameters& parameters)
{
    parseParameters(parameters);
}

void ObstacleDilation::parseParameters(const Parameters& parameters)
{
    cellSize_ = parameters.cellSize;
    dilationSizeF_ = parameters.dilationSize;
    UASSERT(cellSize_ > 0.0f);
    UASSERT(dilationSizeF_ >= 0.0f);

    dilationSize_ = std::ceil(dilationSizeF_ / cellSize_);
    dilationRadiusSqr_ = (dilationSize_ + 0.5f) * (dilationSize_ + 0.5f);
    dilationWidth_ = 2 * dilationSize_ + 1;
    computeDilationPixels();
}

void ObstacleDilation::computeDilationPixels()
{
    dilationPixels_.clear();
    dilationWidthToPixelsNum_.clear();
    std::map<int, std::set<int>> rowToCols;
    int numPixels = 0;
    for (int y = -dilationSize_; y <= dilationSize_; y++)
    {
        for (int x = -dilationSize_; x <= dilationSize_; x++)
        {
            // ((y > 0) - (y < 0)) is equal to sign of y
            float yf = y - ((y > 0) - (y < 0)) * 0.5;
            float xf = x - ((x > 0) - (x < 0)) * 0.5;
            if (yf * yf + xf * xf <= dilationRadiusSqr_)
            {
                rowToCols[y].insert(x);
                numPixels++;
            }
        }
    }
    dilationWidthToPixelsNum_.push_back(0);
    while (numPixels > 0)
    {
        for (auto& [row, cols] : rowToCols)
        {
            if (cols.empty())
            {
                continue;
            }
            auto lastColIt = std::prev(cols.end());
            dilationPixels_.push_back(PixelCoords{row, *lastColIt});
            cols.erase(lastColIt);
            numPixels--;
        }
        dilationWidthToPixelsNum_.push_back(dilationPixels_.size());
    }
}

std::shared_ptr<LocalMap2d> ObstacleDilation::dilate(
    const LocalMap2d& localMap) const
{
    UASSERT(dilationSize_ > 0);
    LocalMap2d::ColoredGrid coloredGrid = localMap.toColoredGrid();

    const MultiArray<std::uint8_t, 2>& grid = coloredGrid.grid;
    MultiArray<std::uint8_t, 2> dilated = grid;
    for (int y = 0; y < grid.shape()[0]; y++)
    {
        int currentDilationWidth = dilationWidth_;
        for (int x = 0; x < grid.shape()[1]; x++)
        {
            currentDilationWidth = std::min(currentDilationWidth + 1, dilationWidth_);
            const std::uint8_t& value = grid[{y, x}];
            if (value != LocalMap2d::ColoredGrid::occupiedCellValue)
            {
                continue;
            }
            for (int i = 0; i < dilationWidthToPixelsNum_[currentDilationWidth]; i++)
            {
                PixelCoords pixelCoords = dilationPixels_[i];
                pixelCoords.y += y;
                pixelCoords.x += x;
                if (!pixelCoords.inFrame(dilated.shape()[0], dilated.shape()[1]))
                {
                    continue;
                }
                dilated[{pixelCoords.y, pixelCoords.x}] = LocalMap2d::ColoredGrid::occupiedCellValue;
            }
            currentDilationWidth = 0;
        }
    }

    coloredGrid.grid = dilated;
    auto dilatedLocalMap = std::make_shared<LocalMap2d>(
        coloredGrid, -1.0f, localMap.pointsDuplicated(),
        localMap.properties());
    return dilatedLocalMap;
}

}
