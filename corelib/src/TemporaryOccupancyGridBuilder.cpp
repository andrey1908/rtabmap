#include <rtabmap/core/TemporaryOccupancyGridBuilder.h>

#include "time_measurer/time_measurer.h"

namespace rtabmap {

TemporaryOccupancyGridBuilder::TemporaryOccupancyGridBuilder(const Parameters& parameters)
{
    parseParameters(parameters);
}

void TemporaryOccupancyGridBuilder::parseParameters(const Parameters& parameters)
{
    cellSize_ = parameters.cellSize;
    temporaryMissProb_ = parameters.temporaryMissProb;
    temporaryHitProb_ = parameters.temporaryHitProb;
    temporaryOccupancyProbThr_ = parameters.temporaryOccupancyProbThr;
    maxTemporaryLocalMaps_ = parameters.maxTemporaryLocalMaps;
    UASSERT(temporaryMissProb_ > 0.0f && temporaryMissProb_ <= 0.5f);
    UASSERT(temporaryHitProb_ >= 0.5f && temporaryHitProb_ < 1.0f);
    UASSERT(temporaryOccupancyProbThr_ > 0.0f && temporaryOccupancyProbThr_ < 1.0f);
    UASSERT(maxTemporaryLocalMaps_ >= 1);

    updated_ = maxTemporaryLocalMaps_ * 2 + 1;
    precomputeProbabilities();
}

void TemporaryOccupancyGridBuilder::precomputeProbabilities()
{
    float missLogit = logodds(temporaryMissProb_);
    float hitLogit = logodds(temporaryHitProb_);
    precomputedProbabilities_.probabilities.resize(maxTemporaryLocalMaps_ + 1, maxTemporaryLocalMaps_ + 1);
    precomputedProbabilities_.probabilitiesThr.resize(maxTemporaryLocalMaps_ + 1, maxTemporaryLocalMaps_ + 1);
    for (int hits = 0; hits <= maxTemporaryLocalMaps_; hits++)
    {
        for (int misses = 0; misses <= maxTemporaryLocalMaps_; misses++)
        {
            float prob = probability(hits * hitLogit + misses * missLogit);
            precomputedProbabilities_.probabilities.coeffRef(hits, misses) = std::lround(prob * 100.0f);
            if (prob >= temporaryOccupancyProbThr_)
            {
                precomputedProbabilities_.probabilitiesThr.coeffRef(hits, misses) = 100;
            }
            else
            {
                precomputedProbabilities_.probabilitiesThr.coeffRef(hits, misses) = 0;
            }
        }
    }
    precomputedProbabilities_.probabilities.coeffRef(0, 0) = -1;
    precomputedProbabilities_.probabilitiesThr.coeffRef(0, 0) = -1;
}

bool TemporaryOccupancyGridBuilder::addLocalMap(
    const Transform& pose, const std::shared_ptr<const LocalMap>& localMap)
{
    MEASURE_BLOCK_TIME(TemporaryOccupancyGridBuilder__addLocalMap);
    UASSERT(localMap);
    TransformedLocalMap transformedLocalMap(*localMap, pose, cellSize_);
    Node node(localMap, std::move(transformedLocalMap));
    nodes_.emplace_back(std::move(node));
    const Node& newNode = nodes_.back();
    MapLimitsI newMapLimits = MapLimitsI::unite(mapLimits_,
        newNode.transformedLocalMap.mapLimits());
    if (mapLimits_ != newMapLimits)
    {
        createOrResizeMap(newMapLimits);
    }
    deployLastNode();
    bool overflowed = false;
    if (nodes_.size() > maxTemporaryLocalMaps_)
    {
        overflowed = true;
        removeFirstNode();
    }
    return overflowed;
}

void TemporaryOccupancyGridBuilder::updatePoses(
    const std::deque<Transform>& updatedPoses)
{
    MEASURE_BLOCK_TIME(TemporaryOccupancyGridBuilder__updatePoses);
    UASSERT(nodes_.size() == updatedPoses.size());
    std::deque<Transform> newPoses;
    {
        auto nodeIt = nodes_.begin();
        auto updatedPoseIt = updatedPoses.begin();
        while (updatedPoseIt != updatedPoses.end())
        {
            const Transform& fromUpdatedPose = nodeIt->localMap->fromUpdatedPose();
            const Transform& updatedPose = *updatedPoseIt;
            newPoses.emplace_back(updatedPose * fromUpdatedPose);
            ++nodeIt;
            ++updatedPoseIt;
        }
    }

    clear();

    MapLimitsI newMapLimits;
    auto nodeIt = nodes_.begin();
    auto newPoseIt = newPoses.begin();
    while (newPoseIt != newPoses.end())
    {
        Node& node = *nodeIt;
        const Transform& newPose = *newPoseIt;
        node.transformedLocalMap.set(*node.localMap, newPose, cellSize_);
        newMapLimits =
            MapLimitsI::unite(newMapLimits, node.transformedLocalMap.mapLimits());
        ++nodeIt;
        ++newPoseIt;
    }
    if (newMapLimits.valid())
    {
        createOrResizeMap(newMapLimits);
        for (const Node& node : nodes_)
        {
            deployNode(node);
        }
    }
}

void TemporaryOccupancyGridBuilder::createOrResizeMap(const MapLimitsI& newMapLimits)
{
    UASSERT(newMapLimits.valid());
    if(!mapLimits_.valid())
    {
        mapLimits_ = newMapLimits;
        int height = newMapLimits.height();
        int width = newMapLimits.width();
        hitCounter_ = CounterType::Constant(height, width, 0);
        missCounter_ = CounterType::Constant(height, width, 0);
        colors_ = ColorsType::Constant(height, width, Color::missingColor.data());
    }
    else if(mapLimits_ != newMapLimits)
    {
        int dstStartY = std::max(mapLimits_.minY() - newMapLimits.minY(), 0);
        int dstStartX = std::max(mapLimits_.minX() - newMapLimits.minX(), 0);
        int srcStartY = std::max(newMapLimits.minY() - mapLimits_.minY(), 0);
        int srcStartX = std::max(newMapLimits.minX() - mapLimits_.minX(), 0);
        MapLimitsI intersection = MapLimitsI::intersect(mapLimits_, newMapLimits);
        int copyHeight = intersection.height();
        int copyWidth = intersection.width();
        UASSERT(copyHeight > 0 && copyWidth > 0);

        int height = newMapLimits.height();
        int width = newMapLimits.width();
        CounterType newHitCounter = CounterType::Constant(height, width, 0);
        CounterType newMissCounter = CounterType::Constant(height, width, 0);
        ColorsType newColors =
            ColorsType::Constant(height, width, Color::missingColor.data());

        newHitCounter.block(dstStartY, dstStartX, copyHeight, copyWidth) =
            hitCounter_.block(srcStartY, srcStartX, copyHeight, copyWidth);
        newMissCounter.block(dstStartY, dstStartX, copyHeight, copyWidth) =
            missCounter_.block(srcStartY, srcStartX, copyHeight, copyWidth);
        newColors.block(dstStartY, dstStartX, copyHeight, copyWidth) =
            colors_.block(srcStartY, srcStartX, copyHeight, copyWidth);

        mapLimits_ = newMapLimits;
        hitCounter_ = std::move(newHitCounter);
        missCounter_ = std::move(newMissCounter);
        colors_ = std::move(newColors);
    }
}

void TemporaryOccupancyGridBuilder::deployLastNode()
{
    UASSERT(nodes_.size());
    deployNode(nodes_.back());
}

void TemporaryOccupancyGridBuilder::deployNode(const Node& node)
{
    UASSERT(node.transformedLocalMap.valid());
    const Eigen::Matrix2Xi& transformedPoints = node.transformedLocalMap.points();
    for (int i = 0; i < transformedPoints.cols(); i++)
    {
        int y = transformedPoints.coeff(1, i) - mapLimits_.minY();
        int x = transformedPoints.coeff(0, i) - mapLimits_.minX();
        UASSERT(y >= 0 && x >= 0 && y < missCounter_.rows() && x < missCounter_.cols());

        if (hitCounter_.coeffRef(y, x) >= updated_)
        {
            continue;
        }
        bool occupied = node.localMap->isObstacle(i);
        if (occupied)
        {
            hitCounter_.coeffRef(y, x) += 1;
        }
        else
        {
            missCounter_.coeffRef(y, x) += 1;
        }
        hitCounter_.coeffRef(y, x) += updated_;

        const Color& color = node.localMap->colors()[i];
        colors_.coeffRef(y, x) = color.data();
    }
    for (int y = 0; y < hitCounter_.rows(); y++)
    {
        for (int x = 0; x < hitCounter_.cols(); x++)
        {
            if (hitCounter_.coeffRef(y, x) >= updated_)
            {
                hitCounter_.coeffRef(y, x) -= updated_;
            }
        }
    }
}

void TemporaryOccupancyGridBuilder::removeFirstNode()
{
    UASSERT(nodes_.size());
    const Node& node = nodes_.front();
    UASSERT(node.transformedLocalMap.valid());
    const Eigen::Matrix2Xi& transformedPoints = node.transformedLocalMap.points();
    for (int i = 0; i < transformedPoints.cols(); i++)
    {
        int y = transformedPoints.coeff(1, i) - mapLimits_.minY();
        int x = transformedPoints.coeff(0, i) - mapLimits_.minX();
        UASSERT(y >= 0 && x >= 0 && y < missCounter_.rows() && x < missCounter_.cols());

        if (hitCounter_.coeffRef(y, x) >= updated_)
        {
            continue;
        }
        bool occupied = node.localMap->isObstacle(i);
        if (occupied)
        {
            hitCounter_.coeffRef(y, x) -= 1;
        }
        else
        {
            missCounter_.coeffRef(y, x) -= 1;
        }
        hitCounter_.coeffRef(y, x) += updated_;
    }
    for (int y = 0; y < hitCounter_.rows(); y++)
    {
        for (int x = 0; x < hitCounter_.cols(); x++)
        {
            if (hitCounter_.coeffRef(y, x) >= updated_)
            {
                hitCounter_.coeffRef(y, x) -= updated_;
            }
            UASSERT(hitCounter_.coeffRef(y, x) >= 0);
            UASSERT(missCounter_.coeffRef(y, x) >= 0);
        }
    }
    nodes_.pop_front();

    MapLimitsI newMapLimits;
    for (const Node& node : nodes_)
    {
        newMapLimits = MapLimitsI::unite(newMapLimits, node.transformedLocalMap.mapLimits());
    }
    createOrResizeMap(newMapLimits);
}

OccupancyGrid TemporaryOccupancyGridBuilder::getOccupancyGrid() const
{
    if (!mapLimits_.valid())
    {
        return OccupancyGrid();
    }
    return getOccupancyGrid(mapLimits_);
}

OccupancyGrid TemporaryOccupancyGridBuilder::getOccupancyGrid(
    const MapLimitsI& roi) const
{
    OccupancyGrid occupancyGrid;
    occupancyGrid.limits = roi;
    occupancyGrid.grid = OccupancyGrid::GridType::Constant(roi.height(), roi.width(), -1);
    MapLimitsI intersection = MapLimitsI::intersect(mapLimits_, roi);
    int height = intersection.height();
    int width = intersection.width();
    if (height == 0 || width == 0)
    {
        return occupancyGrid;
    }
    int dstStartY = std::max(mapLimits_.minY() - roi.minY(), 0);
    int dstStartX = std::max(mapLimits_.minX() - roi.minX(), 0);
    int srcStartY = std::max(roi.minY() - mapLimits_.minY(), 0);
    int srcStartX = std::max(roi.minX() - mapLimits_.minX(), 0);
    for(int y = 0; y < height; ++y)
    {
        for(int x = 0; x < width; ++x)
        {
            int hits = hitCounter_.coeff(y + srcStartY, x + srcStartX);
            int misses = missCounter_.coeff(y + srcStartY, x + srcStartX);
            occupancyGrid.grid.coeffRef(y + dstStartY, x + dstStartX) =
                precomputedProbabilities_.probabilitiesThr.coeff(hits, misses);
        }
    }
    return occupancyGrid;
}

OccupancyGrid TemporaryOccupancyGridBuilder::getProbOccupancyGrid() const
{
    if (!mapLimits_.valid())
    {
        return OccupancyGrid();
    }
    return getProbOccupancyGrid(mapLimits_);
}

OccupancyGrid TemporaryOccupancyGridBuilder::getProbOccupancyGrid(
    const MapLimitsI& roi) const
{
    OccupancyGrid occupancyGrid;
    occupancyGrid.limits = roi;
    occupancyGrid.grid = OccupancyGrid::GridType::Constant(roi.height(), roi.width(), -1);
    MapLimitsI intersection = MapLimitsI::intersect(mapLimits_, roi);
    int height = intersection.height();
    int width = intersection.width();
    if (height == 0 || width == 0)
    {
        return occupancyGrid;
    }
    int dstStartY = std::max(mapLimits_.minY() - roi.minY(), 0);
    int dstStartX = std::max(mapLimits_.minX() - roi.minX(), 0);
    int srcStartY = std::max(roi.minY() - mapLimits_.minY(), 0);
    int srcStartX = std::max(roi.minX() - mapLimits_.minX(), 0);
    for(int y = 0; y < height; ++y)
    {
        for(int x = 0; x < width; ++x)
        {
            int hits = hitCounter_.coeff(y + srcStartY, x + srcStartX);
            int misses = missCounter_.coeff(y + srcStartY, x + srcStartX);
            occupancyGrid.grid.coeffRef(y + dstStartY, x + dstStartX) =
                precomputedProbabilities_.probabilities.coeff(hits, misses);
        }
    }
    return occupancyGrid;
}

ColorGrid TemporaryOccupancyGridBuilder::getColorGrid() const
{
    if (!mapLimits_.valid())
    {
        return ColorGrid();
    }
    return getColorGrid(mapLimits_);
}

ColorGrid TemporaryOccupancyGridBuilder::getColorGrid(
    const MapLimitsI& roi) const
{
    ColorGrid colorGrid;
    colorGrid.limits = roi;
    colorGrid.grid = ColorGrid::GridType::Constant(roi.height(), roi.width(),
        Color::missingColor.data());
    MapLimitsI intersection = MapLimitsI::intersect(mapLimits_, roi);
    int height = intersection.height();
    int width = intersection.width();
    if (height == 0 || width == 0)
    {
        return colorGrid;
    }
    int dstStartY = std::max(mapLimits_.minY() - roi.minY(), 0);
    int dstStartX = std::max(mapLimits_.minX() - roi.minX(), 0);
    int srcStartY = std::max(roi.minY() - mapLimits_.minY(), 0);
    int srcStartX = std::max(roi.minX() - mapLimits_.minX(), 0);
    colorGrid.grid.block(dstStartY, dstStartX, height, width) =
        colors_.block(srcStartY, srcStartX, height, width);
    return colorGrid;
}

void TemporaryOccupancyGridBuilder::clear()
{
    for (Node& node : nodes_)
    {
        node.transformedLocalMap.clear();
    }
    mapLimits_ = MapLimitsI();
    hitCounter_ = CounterType();
    missCounter_ = CounterType();
    colors_ = ColorsType();
}

void TemporaryOccupancyGridBuilder::reset()
{
    nodes_.clear();
    mapLimits_ = MapLimitsI();
    hitCounter_ = CounterType();
    missCounter_ = CounterType();
    colors_ = ColorsType();
}

}
