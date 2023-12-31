#include <rtabmap/core/OccupancyGridMap.h>
#include <rtabmap/core/Serialization.h>

#include <rtabmap/proto/OccupancyGridMap.pb.h>

#include <kas_utils/time_measurer.h>

namespace rtabmap {

OccupancyGridMap::OccupancyGridMap(const Parameters& parameters)
{
    parseParameters(parameters);
}

void OccupancyGridMap::parseParameters(const Parameters& parameters)
{
    UASSERT(parameters.obstacleDilationsParameters.size());

    cellSize_ = parameters.cellSize;
    allowToTransformMap_ = parameters.allowToTransformMap;
    enableObjectTracking_ = parameters.enableObjectTracking;
    enablePosesTrimmer_ = parameters.enablePosesTrimmer;
    UASSERT(cellSize_ > 0.0f);

    posesApproximation_ = std::make_unique<PosesApproximation>(
        parameters.posesApproximationParameters);

    if (enablePosesTrimmer_)
    {
        posesTrimmer_ = std::make_unique<PosesTrimmer>(
            parameters.posesTrimmerParameters);
    }
    else
    {
        posesTrimmer_.reset();
    }

    LocalMapBuilder::Parameters localMapBuilderParameters =
        parameters.localMapBuilderParameters;
    localMapBuilderParameters.cellSize = parameters.cellSize;
    localMapBuilder_ =
        std::make_unique<LocalMapBuilder>(localMapBuilderParameters);

    numBuilders_ = parameters.obstacleDilationsParameters.size();
    obstacleDilations_.clear();
    occupancyGridBuilders_.clear();
    temporaryOccupancyGridBuilders_.clear();
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

    if (enableObjectTracking_)
    {
        objectTracking_ = std::make_unique<ObjectTracking>(cellSize_);
    }
    else
    {
        objectTracking_.reset();
    }
}

std::shared_ptr<LocalMap> OccupancyGridMap::createLocalMap(const SensorData& sensorData,
    const Time& time, const Transform& fromUpdatedPose) const
{
    return localMapBuilder_->createLocalMap(sensorData, time, fromUpdatedPose);
}

int OccupancyGridMap::addLocalMap(const std::shared_ptr<const LocalMap>& localMap)
{
    if (localMap->time() <= skipLocalMapsUpto_)
    {
        return -1;
    }

    int nodeId;
    for (int i = 0; i < numBuilders_; i++)
    {
        std::shared_ptr<const LocalMap> dilatedLocalMap;
        if (obstacleDilations_[i]->dilationSize() > 0.0f)
        {
            MEASURE_BLOCK_TIME(OccupancyGridMap__obstacleDilation);
            dilatedLocalMap = obstacleDilations_[i]->dilate(*localMap);
        }
        else
        {
            dilatedLocalMap = localMap;
        }
        nodeId = occupancyGridBuilders_[i]->addLocalMap(dilatedLocalMap);
    }

    posesApproximation_->addNode(nodeId, localMap->time());

    localMapsWithoutDilation_.emplace(nodeId, localMap);
    return nodeId;
}

int OccupancyGridMap::addLocalMap(const Transform& globalPose,
    const std::shared_ptr<const LocalMap>& localMap)
{
    if (localMap->time() <= skipLocalMapsUpto_)
    {
        return -1;
    }

    int nodeId;
    for (int i = 0; i < numBuilders_; i++)
    {
        std::shared_ptr<const LocalMap> dilatedLocalMap;
        if (obstacleDilations_[i]->dilationSize() > 0.0f)
        {
            MEASURE_BLOCK_TIME(OccupancyGridMap__obstacleDilation);
            dilatedLocalMap = obstacleDilations_[i]->dilate(*localMap);
        }
        else
        {
            dilatedLocalMap = localMap;
        }
        nodeId = occupancyGridBuilders_[i]->addLocalMap(globalPose, dilatedLocalMap);
    }

    const Transform& toUpdatedPose = localMap->fromUpdatedPose().inverse();
    posesApproximation_->addNode(nodeId, localMap->time(), globalPose * toUpdatedPose);

    if (posesTrimmer_)
    {
        posesTrimmer_->addLocalMap(localMap);
    }

    localMapsWithoutDilation_.emplace(nodeId, localMap);
    return nodeId;
}

bool OccupancyGridMap::addTemporaryLocalMap(const Transform& globalPose,
    const std::shared_ptr<const LocalMap>& localMap)
{
    if (localMap->time() <= skipLocalMapsUpto_)
    {
        return false;
    }

    bool overflowed = false;
    for (int i = 0; i < numBuilders_; i++)
    {
        std::shared_ptr<const LocalMap> dilatedLocalMap;
        if (obstacleDilations_[i]->dilationSize() > 0.0f)
        {
            MEASURE_BLOCK_TIME(OccupancyGridMap__obstacleDilation);
            dilatedLocalMap = obstacleDilations_[i]->dilate(*localMap);
        }
        else
        {
            dilatedLocalMap = localMap;
        }
        overflowed =
            temporaryOccupancyGridBuilders_[i]->addLocalMap(globalPose, dilatedLocalMap);
    }

    const Transform& toUpdatedPose = localMap->fromUpdatedPose().inverse();
    posesApproximation_->addTemporaryNode(localMap->time(), globalPose * toUpdatedPose);
    if (overflowed)
    {
        posesApproximation_->removeFirstTemporaryNode();
    }

    if (objectTracking_)
    {
        /// TODO: use local pose for tracking
        objectTracking_->track(*localMap, globalPose);
    }

    return overflowed;
}

bool OccupancyGridMap::updateGlobalToLocal(
    const Transform& localPose, const Transform& globalPose)
{
    Transform newGlobalToLocal = globalPose * localPose.inverse();
    if (globalToLocal_)
    {
        bool equal = Transform::nearlyEqual(*globalToLocal_, newGlobalToLocal);
        if (!equal)
        {
            if (allowToTransformMap_)
            {
                Transform correction = newGlobalToLocal * globalToLocal_->inverse();
                transformMap(correction);
            }
            else
            {
                return false;
            }
        }
    }
    else
    {
        globalToLocal_ = newGlobalToLocal;
    }
    return true;
}

int OccupancyGridMap::addLocalMap(
    const Transform& localPose, const Transform& globalPose,
    const std::shared_ptr<const LocalMap>& localMap)
{
    if (localMap->time() <= skipLocalMapsUpto_)
    {
        return -1;
    }

    bool canProcessLocalMap = updateGlobalToLocal(localPose, globalPose);
    if (!canProcessLocalMap)
    {
        return -1;
    }
    int nodeId = addLocalMap(globalPose, localMap);
    return nodeId;
}

bool OccupancyGridMap::addTemporaryLocalMap(
    const Transform& localPose, const Transform& globalPose,
    const std::shared_ptr<const LocalMap>& localMap)
{
    if (localMap->time() <= skipLocalMapsUpto_)
    {
        return false;
    }

    bool canProcessLocalMap = updateGlobalToLocal(localPose, globalPose);
    if (!canProcessLocalMap)
    {
        return false;
    }
    bool overflowed = addTemporaryLocalMap(globalPose, localMap);
    return overflowed;
}

void OccupancyGridMap::removeNodes(const std::vector<int>& nodeIdsToRemove)
{
    for (int i = 0; i < numBuilders_; i++)
    {
        occupancyGridBuilders_[i]->removeNodes(nodeIdsToRemove);
    }
    posesApproximation_->removeNodes(nodeIdsToRemove);
    for (int nodeIdToRemove : nodeIdsToRemove)
    {
        localMapsWithoutDilation_.erase(nodeIdToRemove);
    }
    /// TODO: maybe remove nodes from poses trimmer
}

void OccupancyGridMap::transformMap(const Transform& transform)
{
    for (int i = 0; i < numBuilders_; i++)
    {
        occupancyGridBuilders_[i]->transformMap(transform);
        temporaryOccupancyGridBuilders_[i]->transformMap(transform);
    }
    posesApproximation_->transformCurrentTrajectory(transform);
    if (globalToLocal_)
    {
        globalToLocal_ = transform * (*globalToLocal_);
    }
}

void OccupancyGridMap::updatePoses(const Trajectories& trajectories,
    const std::optional<Transform>& newGlobalToLocal /* std::nullopt */,
    const Time& skipLocalMapsUpto /* Time() */)
{
    PosesApproximation::ApproximatedPoses approximatedPoses =
        posesApproximation_->approximatePoses(trajectories);
    if (approximatedPoses.temporaryPoses.size() != temporaryNodes(0).size())
    {
        resetTemporary();
    }
    updatePoses(approximatedPoses.poses, approximatedPoses.temporaryPoses,
        approximatedPoses.lastNodeIdToIncludeInCachedMap, newGlobalToLocal,
        skipLocalMapsUpto);
}

void OccupancyGridMap::updatePoses(const std::map<int, Transform>& updatedPoses,
    const std::deque<Transform>& updatedTemporaryPoses,
    int lastNodeIdToIncludeInCachedMap /* -1 */,
    const std::optional<Transform>& newGlobalToLocal /* std::nullopt */,
    const Time& skipLocalMapsUpto /* Time() */)
{
    for (int i = 0; i < numBuilders_; i++)
    {
        occupancyGridBuilders_[i]->updatePoses(updatedPoses, lastNodeIdToIncludeInCachedMap);
        temporaryOccupancyGridBuilders_[i]->updatePoses(updatedTemporaryPoses);
    }
    globalToLocal_ = newGlobalToLocal;
    skipLocalMapsUpto_ = skipLocalMapsUpto;
}

OccupancyGrid OccupancyGridMap::getOccupancyGrid(int index) const
{
    MEASURE_BLOCK_TIME(OccupancyGridMap__getOccupancyGrid);
    UASSERT(index >= 0 && index < numBuilders_);
    const auto& occupancyGridBuilder = occupancyGridBuilders_[index];
    const auto& temporaryOccupancyGridBuilder = temporaryOccupancyGridBuilders_[index];
    MapLimitsI mapLimits = occupancyGridBuilder->mapLimits();
    MapLimitsI temporaryMapLimits = temporaryOccupancyGridBuilder->mapLimits();
    if (!mapLimits.valid() && !temporaryMapLimits.valid())
    {
        return OccupancyGrid();
    }
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
    if (!mapLimits.valid() && !temporaryMapLimits.valid())
    {
        return OccupancyGrid();
    }
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
    if (!mapLimits.valid() && !temporaryMapLimits.valid())
    {
        return ColorGrid();
    }
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
    if (!mapLimits.valid())
    {
        return std::make_pair(
            std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
    }
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
    posesApproximation_->reset();
    globalToLocal_.reset();
    skipLocalMapsUpto_ = Time();
}

void OccupancyGridMap::resetTemporary()
{
    for (int i = 0; i < numBuilders_; i++)
    {
        temporaryOccupancyGridBuilders_[i]->reset();
    }
    posesApproximation_->resetTemporary();
}

void OccupancyGridMap::save(const std::string& file)
{
    MEASURE_BLOCK_TIME(OccupancyGridMap__save);
    MapSerialization writer(file, cellSize_);
    auto localMapIt = localMapsWithoutDilation_.begin();
    const auto& nodesRef = nodes(0);
    auto nodeIt = nodesRef.begin();
    while (localMapIt != localMapsWithoutDilation_.end())
    {
        UASSERT(nodeIt != nodesRef.end());
        UASSERT(localMapIt->first == nodeIt->first);
        int nodeId = nodeIt->first;
        const Node& node = nodeIt->second;
        proto::OccupancyGridMap::Node proto;
        proto.set_node_id(nodeId);
        if (node.hasPose())
        {
            *proto.mutable_global_pose() = rtabmap::toProto(node.pose());
        }
        *proto.mutable_local_map() = rtabmap::toProto(*(localMapIt->second));
        writer.write(proto);
        ++localMapIt;
        ++nodeIt;
    }
    UASSERT(nodeIt == nodesRef.end());
    writer.close();
}

void OccupancyGridMap::load(const std::string& file)
{
    MEASURE_BLOCK_TIME(OccupancyGridMap__load);
    reset();
    MapDeserialization reader(file);
    UASSERT(reader.metaData().version() == MapVersions::mapLatestVersion);
    UASSERT(reader.metaData().cell_size() == cellSize_);
    std::optional<proto::OccupancyGridMap::Node> proto;
    while (proto = reader.read())
    {
        std::shared_ptr<LocalMap> localMap =
            rtabmap::fromProto(proto->local_map());
        if (proto->has_global_pose())
        {
            Transform globalPose = rtabmap::fromProto(proto->global_pose());
            addLocalMap(globalPose, localMap);
        }
        else
        {
            addLocalMap(localMap);
        }
    }
    reader.close();
}

}
