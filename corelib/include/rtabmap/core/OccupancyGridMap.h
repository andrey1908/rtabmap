#pragma once

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/LocalMapBuilder.h>
#include <rtabmap/core/OccupancyGridBuilder.h>
#include <rtabmap/core/TemporaryOccupancyGridBuilder.h>
#include <rtabmap/core/ObstacleDilation.h>
#include <rtabmap/core/ObjectTracking.h>
#include <rtabmap/core/PosesApproximation.h>
#include <rtabmap/core/PosesTrimmer.h>

#include <yaml-cpp/yaml.h>

#include <vector>
#include <memory>
#include <optional>
#include <climits>

namespace rtabmap {

class OccupancyGridMap
{
public:
    struct Parameters
    {
        float cellSize = 0.1f;
        float minNodesTimeDifference = 1.0f;
        bool allowToTransformMap = false;
        bool enablePosesTrimmer = false;
        bool enableObjectTracking = false;

        PosesTrimmer::Parameters posesTrimmerParameters;
        LocalMapBuilder::Parameters localMapBuilderParameters;
        std::vector<ObstacleDilation::Parameters> obstacleDilationsParameters =
            std::vector<ObstacleDilation::Parameters>(1);
        OccupancyGridBuilder::Parameters occupancyGridBuilderParameters;
        TemporaryOccupancyGridBuilder::Parameters
            temporaryOccupancyGridBuilderParameters;

        static Parameters createParameters(const YAML::Node& node)
        {
            UASSERT(node.IsMap());
            Parameters parameters;
            if (node["CellSize"])
            {
                parameters.cellSize = node["CellSize"].as<float>();
            }
            if (node["MinNodesTimeDifference"])
            {
                parameters.minNodesTimeDifference = node["MinNodesTimeDifference"].as<float>();
            }
            if (node["AllowToTransformMap"])
            {
                parameters.allowToTransformMap = node["AllowToTransformMap"].as<bool>();
            }
            if (node["EnablePosesTrimmer"])
            {
                parameters.enablePosesTrimmer = node["EnablePosesTrimmer"].as<bool>();
            }
            if (node["PosesTrimmer"])
            {
                parameters.posesTrimmerParameters =
                    PosesTrimmer::Parameters::createParameters(node["PosesTrimmer"]);
            }
            if (node["EnableObjectTracking"])
            {
                parameters.enableObjectTracking =
                    node["EnableObjectTracking"].as<bool>();
            }
            if (node["LocalMapBuilder"])
            {
                parameters.localMapBuilderParameters =
                    LocalMapBuilder::Parameters::createParameters(
                        node["LocalMapBuilder"]);
            }
            if (node["ObstacleDilation"])
            {
                UASSERT(node["ObstacleDilation"].IsMap() ||
                    node["ObstacleDilation"].IsSequence());
                parameters.obstacleDilationsParameters.clear();
                if (node["ObstacleDilation"].IsMap())
                {
                    parameters.obstacleDilationsParameters.push_back(
                        ObstacleDilation::Parameters::createParameters(
                            node["ObstacleDilation"]));
                }
                if (node["ObstacleDilation"].IsSequence())
                {
                    for (const YAML::Node& obstacleDilationNode : node["ObstacleDilation"])
                    {
                        parameters.obstacleDilationsParameters.push_back(
                            ObstacleDilation::Parameters::createParameters(
                                obstacleDilationNode));
                    }
                }
            }
            if (node["OccupancyGridBuilder"])
            {
                parameters.occupancyGridBuilderParameters =
                    OccupancyGridBuilder::Parameters::createParameters(
                        node["OccupancyGridBuilder"]);
            }
            if (node["TemporaryOccupancyGridBuilder"])
            {
                parameters.temporaryOccupancyGridBuilderParameters =
                    TemporaryOccupancyGridBuilder::Parameters::createParameters(
                        node["TemporaryOccupancyGridBuilder"]);
            }
            return parameters;
        }
    };

public:
    OccupancyGridMap(const Parameters& parameters);
    void parseParameters(const Parameters& parameters);

    std::shared_ptr<LocalMap> createLocalMap(const SensorData& sensorData,
        const Time& time, const Transform& fromUpdatedPose) const;

    int addLocalMap(const std::shared_ptr<const LocalMap>& localMap);

    int addLocalMap(const Transform& globalPose,
        const std::shared_ptr<const LocalMap>& localMap);
    bool addTemporaryLocalMap(const Transform& globalPose,
        const std::shared_ptr<const LocalMap>& localMap);

    int addLocalMap(const Transform& localPose, const Transform& globalPose,
        const std::shared_ptr<const LocalMap>& localMap);
    bool addTemporaryLocalMap(const Transform& localPose, const Transform& globalPose,
        const std::shared_ptr<const LocalMap>& localMap);

    void removeNodes(const std::vector<int>& nodeIdsToRemove);
    void transformMap(const Transform& transform);

    void updatePoses(const Trajectories& trajectories,
        const std::optional<Transform>& newGlobalToLocal,
        const Time& skipLocalMapsUpto = Time());
    void updatePoses(const std::map<int, Transform>& updatedPoses,
        const std::optional<Transform>& newGlobalToLocal,
        int lastNodeIdToIncludeInCachedMap = -1,
        const Time& skipLocalMapsUpto = Time());

    OccupancyGrid getOccupancyGrid(int index) const;
    OccupancyGrid getProbOccupancyGrid(int index) const;
    ColorGrid getColorGrid(int index) const;

    std::set<Time> getPosesToTrim(const Trajectories& trajectories)
        { UASSERT(posesTrimmer_); return posesTrimmer_->getPosesToTrim(trajectories); }

    float cellSize() const { return cellSize_; }
    std::pair<float, float> getGridOrigin(int index) const;
    int maxTemporaryLocalMaps(int index) const
        { return temporaryOccupancyGridBuilders_[index]->maxTemporaryLocalMaps(); }
    const std::map<int, Node>& nodes(int index) const
        { return occupancyGridBuilders_[index]->nodes(); }
    const std::deque<Node>& temporaryNodes(int index) const
        { return temporaryOccupancyGridBuilders_[index]->nodes(); }
    const std::map<int, const std::shared_ptr<const LocalMap>>& localMapsWithoutDilation() const
        { return localMapsWithoutDilation_; }
    const cv::Mat& lastDilatedSemantic() const
        { return localMapBuilder_->lastDilatedSemantic(); }
    int numBuilders() const { return numBuilders_; }
    bool posesTrimmerEnabled() const { return posesTrimmer_ != nullptr; }
    bool objectTrackingEnabled() const { return objectTracking_ != nullptr; }
    const std::vector<ObjectTracking::TrackedObject>& trackedObjects() const
        { UASSERT(objectTracking_); return objectTracking_->trackedObjects(); }
    const std::list<ObjectTracking::MOT16TrackedObject>& mot16TrackedObjectsCache() const
        { UASSERT(objectTracking_); return objectTracking_->mot16TrackedObjectsCache(); }
    const std::vector<LocalMapBuilder::Area>& sensorIgnoreAreas() const
        { return localMapBuilder_->sensorIgnoreAreas(); }

    void reset();
    void resetTemporary();

    void save(const std::string& file);
    void load(const std::string& file);

private:
    bool updateGlobalToLocal(const Transform& localPose, const Transform& globalPose);

private:
    float cellSize_;
    float minNodesTimeDifference_;
    bool allowToTransformMap_;
    bool enableObjectTracking_;
    bool enablePosesTrimmer_;

    std::unique_ptr<PosesApproximation> posesApproximation_;
    std::unique_ptr<PosesTrimmer> posesTrimmer_;
    std::optional<Transform> globalToLocal_;
    Time lastNodeTime_;
    Time skipLocalMapsUpto_;

    std::unique_ptr<LocalMapBuilder> localMapBuilder_;

    int numBuilders_;
    std::vector<std::unique_ptr<ObstacleDilation>> obstacleDilations_;
    std::vector<std::unique_ptr<OccupancyGridBuilder>> occupancyGridBuilders_;
    std::vector<std::unique_ptr<TemporaryOccupancyGridBuilder>>
        temporaryOccupancyGridBuilders_;

    std::unique_ptr<ObjectTracking> objectTracking_;

    std::map<int, const std::shared_ptr<const LocalMap>> localMapsWithoutDilation_;
};

}
