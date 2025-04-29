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
    minNodesTimeDifference_ = parameters.minNodesTimeDifference;
    allowToTransformMap_ = parameters.allowToTransformMap;
    enableObjectTracking_ = parameters.enableObjectTracking;
    enablePosesTrimmer_ = parameters.enablePosesTrimmer;
    UASSERT(cellSize_ > 0.0f);
    UASSERT(minNodesTimeDifference_ >= 0.0f);

    posesApproximation_ = std::make_unique<PosesApproximation>();

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
    temporaryOccupancyGridBuilders_.clear();
    occupancyGridBuilders_.clear();
    for (int i = 0; i < numBuilders_; i++)
    {
        ObstacleDilation::Parameters obstacleDilationParameters =
            parameters.obstacleDilationsParameters[i];
        obstacleDilationParameters.cellSize = parameters.cellSize;
        obstacleDilations_.push_back(
            std::make_unique<ObstacleDilation>(obstacleDilationParameters));

        TemporaryOccupancyGridBuilder::Parameters temporaryOccupancyGridBuilderParameters =
            parameters.temporaryOccupancyGridBuilderParameters;
        temporaryOccupancyGridBuilderParameters.cellSize = parameters.cellSize;
        temporaryOccupancyGridBuilders_.push_back(
            std::make_unique<TemporaryOccupancyGridBuilder>(
                temporaryOccupancyGridBuilderParameters));

        OccupancyGridBuilder::Parameters occupancyGridBuilderParameters =
            parameters.occupancyGridBuilderParameters;
        occupancyGridBuilderParameters.cellSize = parameters.cellSize;
        occupancyGridBuilders_.push_back(
            std::make_unique<OccupancyGridBuilder>(occupancyGridBuilderParameters));
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

bool OccupancyGridMap::localMapCanBeAdded(const Time& time)
{
    if (time <= skipLocalMapsUpto_)
    {
        return false;
    }

    double timeDiff = time.toSec() - lastNodeTime_.toSec();
    if (timeDiff >= 0.0 && timeDiff < minNodesTimeDifference_)
    {
        return false;
    }

    return true;
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

    double timeDiff = localMap->time().toSec() - lastNodeTime_.toSec();
    if (timeDiff < 0.0)
    {
        lastNodeTime_ = Time();
    }
    else if (timeDiff < minNodesTimeDifference_)
    {
        return -1;
    }
    else
    {
        lastNodeTime_ = localMap->time();
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

    posesApproximation_->addTime(nodeId, localMap->time());

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

    if (objectTracking_)
    {
        /// TODO: use local pose for tracking
        objectTracking_->track(*localMap, globalPose);
    }

    return overflowed;
}

int OccupancyGridMap::addLocalMap(const Transform& globalPose,
    const std::shared_ptr<const LocalMap>& localMap)
{
    if (localMap->time() <= skipLocalMapsUpto_)
    {
        return -1;
    }

    double timeDiff = localMap->time().toSec() - lastNodeTime_.toSec();
    if (timeDiff < 0.0)
    {
        lastNodeTime_ = Time();
    }
    else if (timeDiff < minNodesTimeDifference_)
    {
        return -1;
    }
    else
    {
        lastNodeTime_ = localMap->time();
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

    posesApproximation_->addTime(nodeId, localMap->time());
    if (posesTrimmer_)
    {
        posesTrimmer_->addLocalMap(localMap);
    }

    localMapsWithoutDilation_.emplace(nodeId, localMap);
    return nodeId;
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

bool OccupancyGridMap::addTemporaryLocalMap(
    const Transform& localPose, const Transform& globalPose,
    const std::shared_ptr<const LocalMap>& localMap)
{
    posesApproximation_->addLocalPose(localMap->time(), localPose);

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

int OccupancyGridMap::addLocalMap(
    const Transform& localPose, const Transform& globalPose,
    const std::shared_ptr<const LocalMap>& localMap)
{
    posesApproximation_->addLocalPose(localMap->time(), localPose);

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

void OccupancyGridMap::removeNodes(const std::vector<int>& nodeIdsToRemove)
{
    for (int i = 0; i < numBuilders_; i++)
    {
        occupancyGridBuilders_[i]->removeNodes(nodeIdsToRemove);
    }
    posesApproximation_->removeTimes(nodeIdsToRemove);
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
        temporaryOccupancyGridBuilders_[i]->transformMap(transform);
        occupancyGridBuilders_[i]->transformMap(transform);
    }
    if (globalToLocal_)
    {
        globalToLocal_ = transform * (*globalToLocal_);
    }
}

void OccupancyGridMap::updatePoses(
    const Trajectories& trajectories,
    const std::optional<Transform>& newGlobalToLocal,
    const Time& skipLocalMapsUpto /* Time() */)
{
    const auto& [updatedPoses, lastNodeIdToIncludeInCachedMap] =
        posesApproximation_->approximatePoses(trajectories);
    updatePoses(updatedPoses, newGlobalToLocal,
        lastNodeIdToIncludeInCachedMap, skipLocalMapsUpto);
}

void OccupancyGridMap::updatePoses(
    const std::map<int, Transform>& updatedPoses,
    const std::optional<Transform>& newGlobalToLocal,
    int lastNodeIdToIncludeInCachedMap /* -1 */,
    const Time& skipLocalMapsUpto /* Time() */)
{
    bool resetTemporary = !newGlobalToLocal.has_value() || temporaryNodes(0).empty();
    UASSERT(resetTemporary || globalToLocal_.has_value());

    Transform correction;
    if (!resetTemporary)
    {
        correction = *newGlobalToLocal * globalToLocal_->inverse();
    }
    for (int i = 0; i < numBuilders_; i++)
    {
        if (resetTemporary)
        {
            temporaryOccupancyGridBuilders_[i]->reset();
        }
        else
        {
            temporaryOccupancyGridBuilders_[i]->transformMap(correction);
        }
        occupancyGridBuilders_[i]->updatePoses(updatedPoses, lastNodeIdToIncludeInCachedMap);
    }

    globalToLocal_ = newGlobalToLocal;
    skipLocalMapsUpto_ = skipLocalMapsUpto;
}

OccupancyGrid OccupancyGridMap::getOccupancyGrid(int index) const
{
    MEASURE_BLOCK_TIME(OccupancyGridMap__getOccupancyGrid);
    UASSERT(index >= 0 && index < numBuilders_);
    const auto& temporaryOccupancyGridBuilder = temporaryOccupancyGridBuilders_[index];
    const auto& occupancyGridBuilder = occupancyGridBuilders_[index];
    MapLimitsI temporaryMapLimits = temporaryOccupancyGridBuilder->mapLimits();
    MapLimitsI mapLimits = occupancyGridBuilder->mapLimits();
    if (!temporaryMapLimits.valid() && !mapLimits.valid())
    {
        return OccupancyGrid();
    }
    if (!temporaryMapLimits.valid())
    {
        return occupancyGridBuilder->getOccupancyGrid();
    }
    if (!mapLimits.valid())
    {
        return temporaryOccupancyGridBuilder->getOccupancyGrid();
    }

    MapLimitsI combinedMapLimits = MapLimitsI::unite(temporaryMapLimits, mapLimits);
    OccupancyGrid temporaryOccupancyGrid = temporaryOccupancyGridBuilder->getOccupancyGrid();
    OccupancyGrid occupancyGrid = occupancyGridBuilder->getOccupancyGrid(combinedMapLimits);

    int dstStartY = temporaryOccupancyGrid.limits.min()[0] - occupancyGrid.limits.min()[0];
    int dstStartX = temporaryOccupancyGrid.limits.min()[1] - occupancyGrid.limits.min()[1];
    int height = temporaryOccupancyGrid.limits.shape()[0];
    int width = temporaryOccupancyGrid.limits.shape()[1];
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
    const auto& temporaryOccupancyGridBuilder = temporaryOccupancyGridBuilders_[index];
    const auto& occupancyGridBuilder = occupancyGridBuilders_[index];
    MapLimitsI temporaryMapLimits = temporaryOccupancyGridBuilder->mapLimits();
    MapLimitsI mapLimits = occupancyGridBuilder->mapLimits();
    if (!temporaryMapLimits.valid() && !mapLimits.valid())
    {
        return OccupancyGrid();
    }
    if (!temporaryMapLimits.valid())
    {
        return occupancyGridBuilder->getProbOccupancyGrid();
    }
    if (!mapLimits.valid())
    {
        return temporaryOccupancyGridBuilder->getProbOccupancyGrid();
    }

    MapLimitsI combinedMapLimits = MapLimitsI::unite(temporaryMapLimits, mapLimits);
    OccupancyGrid temporaryOccupancyGrid = temporaryOccupancyGridBuilder->getProbOccupancyGrid();
    OccupancyGrid occupancyGrid = occupancyGridBuilder->getProbOccupancyGrid(combinedMapLimits);

    int dstStartY = temporaryOccupancyGrid.limits.min()[0] - occupancyGrid.limits.min()[0];
    int dstStartX = temporaryOccupancyGrid.limits.min()[1] - occupancyGrid.limits.min()[1];
    int height = temporaryOccupancyGrid.limits.shape()[0];
    int width = temporaryOccupancyGrid.limits.shape()[1];
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
    const auto& temporaryOccupancyGridBuilder = temporaryOccupancyGridBuilders_[index];
    const auto& occupancyGridBuilder = occupancyGridBuilders_[index];
    MapLimitsI temporaryMapLimits = temporaryOccupancyGridBuilder->mapLimits();
    MapLimitsI mapLimits = occupancyGridBuilder->mapLimits();
    if (!temporaryMapLimits.valid() && !mapLimits.valid())
    {
        return ColorGrid();
    }
    if (!temporaryMapLimits.valid())
    {
        return occupancyGridBuilder->getColorGrid();
    }
    if (!mapLimits.valid())
    {
        return temporaryOccupancyGridBuilder->getColorGrid();
    }

    MapLimitsI combinedMapLimits = MapLimitsI::unite(temporaryMapLimits, mapLimits);
    ColorGrid temporaryColorGrid = temporaryOccupancyGridBuilder->getColorGrid();
    ColorGrid colorGrid = occupancyGridBuilder->getColorGrid(combinedMapLimits);

    int dstStartY = temporaryColorGrid.limits.min()[0] - colorGrid.limits.min()[0];
    int dstStartX = temporaryColorGrid.limits.min()[1] - colorGrid.limits.min()[1];
    int height = temporaryColorGrid.limits.shape()[0];
    int width = temporaryColorGrid.limits.shape()[1];
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
    MapLimitsI mapLimits = MapLimitsI::unite(
        temporaryOccupancyGridBuilders_[index]->mapLimits(),
        occupancyGridBuilders_[index]->mapLimits());
    if (!mapLimits.valid())
    {
        return std::make_pair(
            std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
    }
    float originX = mapLimits.min()[1] * cellSize_;
    float originY = mapLimits.min()[0] * cellSize_;
    return std::make_pair(originX, originY);
}

void OccupancyGridMap::reset()
{
    for (int i = 0; i < numBuilders_; i++)
    {
        temporaryOccupancyGridBuilders_[i]->reset();
        occupancyGridBuilders_[i]->reset();
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
}

void OccupancyGridMap::save(const std::string& file)
{
    MEASURE_BLOCK_TIME(OccupancyGridMap__save);
    MapSerialization writer(file, cellSize_, posesApproximation_->localPoses());
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

    for (const TimedPose& timedPose : fromProto(reader.localPoses()))
    {
        posesApproximation_->addLocalPose(timedPose.time, timedPose.pose);
    }

    std::optional<proto::OccupancyGridMap::Node> proto;
    while (proto = reader.read())
    {
        std::shared_ptr<LocalMap> localMap = rtabmap::fromProto(proto->local_map());
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
