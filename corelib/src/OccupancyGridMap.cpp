#include <rtabmap/core/OccupancyGridMap.h>

#include "time_measurer/time_measurer.h"

namespace rtabmap {

OccupancyGridMap::OccupancyGridMap(const Parameters& parameters)
{
    parseParameters(parameters);
}

void OccupancyGridMap::parseParameters(const Parameters& parameters)
{
    UASSERT(parameters.obstacleDilationsParameters.size());
    cellSize_ = parameters.cellSize;

    LocalMapBuilder::Parameters localMapBuilderParameters =
        parameters.localMapBuilderParameters;
    localMapBuilderParameters.cellSize = parameters.cellSize;
    localMapBuilder_ =
        std::make_unique<LocalMapBuilder>(localMapBuilderParameters);

    numBuilders_ = parameters.obstacleDilationsParameters.size();
    for (int i = 0; i < numBuilders_; i++)
    {
        ObstacleDilation::Parameters obstacleDilationParameters =
            parameters.obstacleDilationsParameters[i];
        obstacleDilationParameters.cellSize = parameters.cellSize;
        obstacleDilations_.push_back(
            std::make_unique<ObstacleDilation>(obstacleDilationParameters));

        OccupancyGridBuilder::Parameters occupancyGridBuilderParameters =
            parameters.occupancyGridBuilderParameters;
        occupancyGridBuilderParameters.cellSize = parameters.cellSize;
        occupancyGridBuilders_.push_back(
            std::make_unique<OccupancyGridBuilder>(occupancyGridBuilderParameters));

        TemporaryOccupancyGridBuilder::Parameters temporaryOccupancyGridBuilderParameters =
            parameters.temporaryOccupancyGridBuilderParameters;
        temporaryOccupancyGridBuilderParameters.cellSize = parameters.cellSize;
        temporaryOccupancyGridBuilders_.push_back(
            std::make_unique<TemporaryOccupancyGridBuilder>(
                temporaryOccupancyGridBuilderParameters));
    }
}

std::shared_ptr<LocalMap> OccupancyGridMap::createLocalMap(
    const Signature& signature,
    const Transform& fromUpdatedPose /* Transform::getIdentity() */) const
{
    UASSERT(!fromUpdatedPose.isNull());
    std::shared_ptr<LocalMap> localMap = localMapBuilder_->createLocalMap(signature);
    localMap->setFromUpdatedPose(fromUpdatedPose);
    return localMap;
}

void OccupancyGridMap::addLocalMap(int nodeId,
    std::shared_ptr<const LocalMap> localMap)
{
    localMapsWithoutObstacleDilation_.emplace(nodeId, localMap);
    for (int i = 0; i < numBuilders_; i++)
    {
        std::shared_ptr<const LocalMap> dilatedLocalMap;
        if (obstacleDilations_[i]->dilationSize() != 0.0f)
        {
            MEASURE_BLOCK_TIME(OccupancyGridMap__obstacleDilation);
            dilatedLocalMap = obstacleDilations_[i]->dilate(*localMap);
        }
        else
        {
            dilatedLocalMap = localMap;
        }
        occupancyGridBuilders_[i]->addLocalMap(nodeId, dilatedLocalMap);
    }
}

void OccupancyGridMap::addLocalMap(int nodeId, const Transform& pose,
    std::shared_ptr<const LocalMap> localMap)
{
    localMapsWithoutObstacleDilation_.emplace(nodeId, localMap);
    for (int i = 0; i < numBuilders_; i++)
    {
        std::shared_ptr<const LocalMap> dilatedLocalMap;
        if (obstacleDilations_[i]->dilationSize() != 0.0f)
        {
            MEASURE_BLOCK_TIME(OccupancyGridMap__obstacleDilation);
            dilatedLocalMap = obstacleDilations_[i]->dilate(*localMap);
        }
        else
        {
            dilatedLocalMap = localMap;
        }
        occupancyGridBuilders_[i]->addLocalMap(nodeId, pose, dilatedLocalMap);
    }
}

void OccupancyGridMap::addTemporaryLocalMap(const Transform& pose,
    std::shared_ptr<const LocalMap> localMap)
{
    std::shared_ptr<const LocalMap> dilatedLocalMap;
    for (int i = 0; i < numBuilders_; i++)
    {
        if (obstacleDilations_[i]->dilationSize() != 0.0f)
        {
            MEASURE_BLOCK_TIME(OccupancyGridMap__obstacleDilation);
            dilatedLocalMap = obstacleDilations_[i]->dilate(*localMap);
        }
        else
        {
            dilatedLocalMap = localMap;
        }
        temporaryOccupancyGridBuilders_[i]->addLocalMap(pose, dilatedLocalMap);
    }
}

void OccupancyGridMap::updatePoses(const std::map<int, Transform>& updatedPoses,
        const std::deque<Transform>& updatedTemporaryPoses,
        int lastNodeIdToIncludeInCachedMap /* -1 */)
{
    for (int i = 0; i < numBuilders_; i++)
    {
        occupancyGridBuilders_[i]->updatePoses(updatedPoses, lastNodeIdToIncludeInCachedMap);
        temporaryOccupancyGridBuilders_[i]->updatePoses(updatedTemporaryPoses);
    }
}

OccupancyGrid OccupancyGridMap::getOccupancyGrid(int index) const
{
    MEASURE_BLOCK_TIME(OccupancyGridMap__getOccupancyGrid);
    UASSERT(index >= 0 && index < numBuilders_);
    const auto& occupancyGridBuilder = occupancyGridBuilders_[index];
    const auto& temporaryOccupancyGridBuilder = temporaryOccupancyGridBuilders_[index];
    MapLimitsI mapLimits = occupancyGridBuilder->mapLimits();
    MapLimitsI temporaryMapLimits = temporaryOccupancyGridBuilder->mapLimits();
    UASSERT(mapLimits.valid() || temporaryMapLimits.valid());
    if (!mapLimits.valid())
    {
        return temporaryOccupancyGridBuilder->getOccupancyGrid();
    }
    if (!temporaryMapLimits.valid())
    {
        return occupancyGridBuilder->getOccupancyGrid();
    }

    MapLimitsI combinedMapLimits =
        MapLimitsI::unite(mapLimits, temporaryMapLimits);
    OccupancyGrid occupancyGrid =
        occupancyGridBuilder->getOccupancyGrid(combinedMapLimits);
    OccupancyGrid temporaryOccupancyGrid =
        temporaryOccupancyGridBuilder->getOccupancyGrid();

    int dstStartY = temporaryOccupancyGrid.limits.minY() - occupancyGrid.limits.minY();
    int dstStartX = temporaryOccupancyGrid.limits.minX() - occupancyGrid.limits.minX();
    int height = temporaryOccupancyGrid.limits.height();
    int width = temporaryOccupancyGrid.limits.width();
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            char value = temporaryOccupancyGrid.grid.coeff(y, x);
            if (value != -1)
            {
                occupancyGrid.grid.coeffRef(y + dstStartY, x + dstStartX) = value;
            }
        }
    }
    return occupancyGrid;
}

OccupancyGrid OccupancyGridMap::getProbOccupancyGrid(int index) const
{
    MEASURE_BLOCK_TIME(OccupancyGridMap__getProbOccupancyGrid);
    UASSERT(index >= 0 && index < numBuilders_);
    const auto& occupancyGridBuilder = occupancyGridBuilders_[index];
    const auto& temporaryOccupancyGridBuilder = temporaryOccupancyGridBuilders_[index];
    MapLimitsI mapLimits = occupancyGridBuilder->mapLimits();
    MapLimitsI temporaryMapLimits = temporaryOccupancyGridBuilder->mapLimits();
    UASSERT(mapLimits.valid() || temporaryMapLimits.valid());
    if (!mapLimits.valid())
    {
        return temporaryOccupancyGridBuilder->getProbOccupancyGrid();
    }
    if (!temporaryMapLimits.valid())
    {
        return occupancyGridBuilder->getProbOccupancyGrid();
    }

    MapLimitsI combinedMapLimits =
        MapLimitsI::unite(mapLimits, temporaryMapLimits);
    OccupancyGrid occupancyGrid =
        occupancyGridBuilder->getProbOccupancyGrid(combinedMapLimits);
    OccupancyGrid temporaryOccupancyGrid =
        temporaryOccupancyGridBuilder->getProbOccupancyGrid();

    int dstStartY = temporaryOccupancyGrid.limits.minY() - occupancyGrid.limits.minY();
    int dstStartX = temporaryOccupancyGrid.limits.minX() - occupancyGrid.limits.minX();
    int height = temporaryOccupancyGrid.limits.height();
    int width = temporaryOccupancyGrid.limits.width();
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            char value = temporaryOccupancyGrid.grid.coeff(y, x);
            if (value != -1)
            {
                occupancyGrid.grid.coeffRef(y + dstStartY, x + dstStartX) = value;
            }
        }
    }
    return occupancyGrid;
}

ColorGrid OccupancyGridMap::getColorGrid(int index) const
{
    MEASURE_BLOCK_TIME(OccupancyGridMap__getColorGrid);
    UASSERT(index >= 0 && index < numBuilders_);
    const auto& occupancyGridBuilder = occupancyGridBuilders_[index];
    const auto& temporaryOccupancyGridBuilder = temporaryOccupancyGridBuilders_[index];
    MapLimitsI mapLimits = occupancyGridBuilder->mapLimits();
    MapLimitsI temporaryMapLimits = temporaryOccupancyGridBuilder->mapLimits();
    UASSERT(mapLimits.valid() || temporaryMapLimits.valid());
    if (!mapLimits.valid())
    {
        return temporaryOccupancyGridBuilder->getColorGrid();
    }
    if (!temporaryMapLimits.valid())
    {
        return occupancyGridBuilder->getColorGrid();
    }

    MapLimitsI combinedMapLimits =
        MapLimitsI::unite(mapLimits, temporaryMapLimits);
    ColorGrid colorGrid =
        occupancyGridBuilder->getColorGrid(combinedMapLimits);
    ColorGrid temporaryColorGrid =
        temporaryOccupancyGridBuilder->getColorGrid();

    int dstStartY = temporaryColorGrid.limits.minY() - colorGrid.limits.minY();
    int dstStartX = temporaryColorGrid.limits.minX() - colorGrid.limits.minX();
    int height = temporaryColorGrid.limits.height();
    int width = temporaryColorGrid.limits.width();
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            int value = temporaryColorGrid.grid.coeff(y, x);
            if (value != Color::missingColor.data())
            {
                colorGrid.grid.coeffRef(y + dstStartY, x + dstStartX) = value;
            }
        }
    }
    return colorGrid;
}

std::pair<float, float> OccupancyGridMap::getGridOrigin(int index) const
{
    UASSERT(index >= 0 && index < numBuilders_);
    MapLimitsI mapLimits = MapLimitsI::unite(occupancyGridBuilders_[index]->mapLimits(),
        temporaryOccupancyGridBuilders_[index]->mapLimits());
    UASSERT(mapLimits.valid());
    float originX = mapLimits.minX() * cellSize_;
    float originY = mapLimits.minY() * cellSize_;
    return std::make_pair(originX, originY);
}

void OccupancyGridMap::reset()
{
    for (int i = 0; i < numBuilders_; i++)
    {
        occupancyGridBuilders_[i]->reset();
        temporaryOccupancyGridBuilders_[i]->reset();
    }
}

}
