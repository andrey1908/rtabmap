#include <rtabmap/core/LocalMap.h>
#include <rtabmap/core/Compression.h>

#include <rtabmap/utilite/ULogger.h>

#include <kas_utils/utils.hpp>

using kas_utils::castArray;


namespace rtabmap {

template <int Dims>
LocalMap<Dims>::LocalMap() :
    numObstacles_(0),
    numMaybeEmpty_(0),
    numEmpty_(0) {}

template <int Dims>
LocalMap<Dims>::LocalMap(const Properties& properties) :
    numObstacles_(0),
    numMaybeEmpty_(0),
    numEmpty_(0),
    properties_(properties) {}

template <int Dims>
LocalMap<Dims>::LocalMap(const ColoredGrid& coloredGrid,
    float maxRange2dSqr, bool duplicatePoints)
{
    fromColoredGrid(coloredGrid, maxRange2dSqr, duplicatePoints);
}

template <int Dims>
LocalMap<Dims>::LocalMap(const ColoredGrid& coloredGrid,
        float maxRange2dSqr, bool duplicatePoints,
        const Properties& properties) :
    properties_(properties)
{
    fromColoredGrid(coloredGrid, maxRange2dSqr, duplicatePoints);
}

template <int Dims>
void LocalMap<Dims>::fromColoredGrid(const ColoredGrid& coloredGrid,
    float maxRange2dSqr, bool duplicatePoints)
{
    UASSERT(coloredGrid.cellSize > 0.0f);
    UASSERT(coloredGrid.limits.valid());
    UASSERT(coloredGrid.grid.shape()[0] == coloredGrid.limits.shape()[0]);
    UASSERT(coloredGrid.grid.shape()[1] == coloredGrid.limits.shape()[1]);
    UASSERT(coloredGrid.colors.shape()[0] == coloredGrid.limits.shape()[0]);
    UASSERT(coloredGrid.colors.shape()[1] == coloredGrid.limits.shape()[1]);

    const float& cellSize = coloredGrid.cellSize;
    const std::array<int, Dims>& limitsMin = coloredGrid.limits.min();

    std::vector<std::array<int, Dims>> occupiedCells;
    std::vector<std::array<int, Dims>> maybeEmptyCells;
    std::vector<std::array<int, Dims>> emptyCells;
    std::array<std::size_t, Dims> beginIndex = {};
    auto gridBlock = coloredGrid.grid.block(beginIndex, coloredGrid.grid.shape());
    for (auto it = gridBlock.begin(); it != gridBlock.end(); ++it)
    {
        std::array<int, Dims> cell = castArray<int>(it.position());
        if (maxRange2dSqr >= 0.0f)
        {
            float rangeSqr = 0.f;
            for (int d = 0; d < 2; d++)  // i < 2 because maxRange2dSqr parameter has '2d' in its name
            {
                float axisRange = (cell[d] + limitsMin[d] + 0.5f) * cellSize;
                rangeSqr += axisRange * axisRange;
            }
            if (rangeSqr > maxRange2dSqr)
            {
                continue;
            }
        }
        const std::uint8_t& value = *it;
        switch (value)
        {
        case ColoredGrid::occupiedCellValue:
            occupiedCells.emplace_back(cell);
            break;
        case ColoredGrid::maybeEmptyCellValue:
            maybeEmptyCells.emplace_back(cell);
            break;
        case ColoredGrid::emptyCellValue:
            emptyCells.emplace_back(cell);
            break;
        }
    }

    int numPoints = occupiedCells.size() + maybeEmptyCells.size() + emptyCells.size();
    colors_.clear();
    if (!duplicatePoints)
    {
        numObstacles_ = occupiedCells.size();
        numMaybeEmpty_ = maybeEmptyCells.size();
        numEmpty_ = emptyCells.size();
        points_.resize(3, numPoints);
        colors_.reserve(numPoints);
    }
    else
    {
        numObstacles_ = occupiedCells.size() * 2;
        numMaybeEmpty_ = maybeEmptyCells.size() * 2;
        numEmpty_ = emptyCells.size() * 2;
        points_.resize(3, numPoints * 2);
        colors_.reserve(numPoints * 2);
    }

    std::vector<int> accumPointsNum = {
        occupiedCells.size(),
        occupiedCells.size() + maybeEmptyCells.size()};
    for (int i = 0; i < numPoints; i++)
    {
        std::array<int, Dims> cell;
        if (i < accumPointsNum[0])
        {
            cell = occupiedCells[i];
        }
        else if (i < accumPointsNum[1])
        {
            cell = maybeEmptyCells[i - accumPointsNum[0]];
        }
        else
        {
            cell = emptyCells[i - accumPointsNum[1]];
        }

        const Color& color =
            reinterpret_cast<const Color&>(coloredGrid.colors[castArray<std::size_t>(cell)]);
        if (!duplicatePoints)
        {
            for (int d = 0; d < Dims; d++)
            {
                points_(Dims - 1 - d, i) = (cell[d] + limitsMin[d] + 0.5f) * cellSize;
            }
            if constexpr(Dims == 2)
            {
                points_(2, i) = 0.0f;
            }
            colors_.push_back(color);
        }
        else
        {
            for (int d = 0; d < Dims; d++)
            {
                points_(Dims - 1 - d, i * 2) = (cell[d] + limitsMin[d] + 0.25f) * cellSize;
            }
            if constexpr(Dims == 2)
            {
                points_(2, i * 2) = 0.0f;
            }
            colors_.push_back(color);

            for (int d = 0; d < Dims; d++)
            {
                points_(Dims - 1 - d, i * 2 + 1) = (cell[d] + limitsMin[d] + 0.75f) * cellSize;
            }
            if constexpr(Dims == 2)
            {
                points_(2, i * 2 + 1) = 0.0f;
            }
            colors_.push_back(color);
        }
    }
    pointsDuplicated_ = duplicatePoints;
    cellSize_ = coloredGrid.cellSize;
    limits_ = coloredGrid.limits;
}

template <int Dims>
typename LocalMap<Dims>::ColoredGrid LocalMap<Dims>::toColoredGrid() const
{
    UASSERT(limits_.valid());
    ColoredGrid coloredGrid;
    coloredGrid.cellSize = cellSize_;
    coloredGrid.limits = limits_;
    coloredGrid.grid.reset(castArray<std::size_t>(limits_.shape()));
    coloredGrid.colors.reset(castArray<std::size_t>(limits_.shape()));

    const std::array<int, Dims>& limitsMin = limits_.min();
    int step = pointsDuplicated_ ? 2 : 1;
    for (int i = 0; i < points_.cols(); i += step)
    {
        std::array<int, Dims> cell;
        for (int d = 0; d < Dims; d++)
        {
            cell[Dims - 1 - d] = std::floor(points_.coeff(d, i) / cellSize_) - limitsMin[d];
        }
        std::uint8_t& value = coloredGrid.grid[castArray<std::size_t>(cell)];
        PointType pointType = getPointType(i);
        switch (pointType)
        {
        case PointType::Occupied:
            value = LocalMap::ColoredGrid::occupiedCellValue;
            break;
        case PointType::MaybeEmpty:
            value = LocalMap::ColoredGrid::maybeEmptyCellValue;
            break;
        case PointType::Empty:
            value = LocalMap::ColoredGrid::emptyCellValue;
            break;
        }
        coloredGrid.colors[castArray<std::size_t>(cell)] = colors_[i].data();
    }
    return coloredGrid;
}

template <int Dims>
proto::LocalMap::ColoredGrid toProto(const typename LocalMap<Dims>::ColoredGrid& coloredGrid)
{
    proto::LocalMap::ColoredGrid proto;
    proto.set_cell_size(coloredGrid.cellSize);
    *proto.mutable_limits() = toProto(coloredGrid.limits);
    proto.set_grid_compressed(compressMultiArray(coloredGrid.grid));
    proto.set_colors_compressed(compressMultiArray(coloredGrid.colors));
    return proto;
}

template <int Dims>
typename LocalMap<Dims>::ColoredGrid fromProto(const proto::LocalMap::ColoredGrid& proto)
{
    typename LocalMap<Dims>::ColoredGrid coloredGrid;
    coloredGrid.cellSize = proto.cell_size();
    coloredGrid.limits = fromProto<int, Dims>(proto.limits());
    coloredGrid.grid = decompressMultiArray<std::uint8_t, Dims>(proto.grid_compressed());
    coloredGrid.colors = decompressMultiArray<std::int32_t, Dims>(proto.colors_compressed());
    return coloredGrid;
}

template <int Dims>
proto::LocalMap toProto(const LocalMap<Dims>& localMap)
{
    proto::LocalMap proto;
    *proto.mutable_colored_grid() = toProto<Dims>(localMap.toColoredGrid());
    UASSERT(!localMap.fromUpdatedPose().isNull());
    *proto.mutable_from_updated_pose() = toProto(localMap.fromUpdatedPose());
    *proto.mutable_time() = toProto(localMap.time());
    proto.set_points_duplicated(localMap.pointsDuplicated());
    return proto;
}

template <int Dims>
std::shared_ptr<LocalMap<Dims>> fromProto(const proto::LocalMap& proto)
{
    auto localMap = std::make_shared<LocalMap<Dims>>(fromProto<Dims>(proto.colored_grid()),
        -1.0f, proto.points_duplicated());
    localMap->setFromUpdatedPose(fromProto(proto.from_updated_pose()));
    localMap->setTime(fromProto(proto.time()));
    return localMap;
}

template class LocalMap<2>;
template class LocalMap<3>;

template proto::LocalMap::ColoredGrid toProto<2>(const typename LocalMap<2>::ColoredGrid& coloredGrid);
template proto::LocalMap::ColoredGrid toProto<3>(const typename LocalMap<3>::ColoredGrid& coloredGrid);
template typename LocalMap<2>::ColoredGrid fromProto<2>(const proto::LocalMap::ColoredGrid& proto);
template typename LocalMap<3>::ColoredGrid fromProto<3>(const proto::LocalMap::ColoredGrid& proto);

template proto::LocalMap toProto(const LocalMap<2>& localMap);
template proto::LocalMap toProto(const LocalMap<3>& localMap);
template std::shared_ptr<LocalMap<2>> fromProto<2>(const proto::LocalMap& proto);
template std::shared_ptr<LocalMap<3>> fromProto<3>(const proto::LocalMap& proto);

}
