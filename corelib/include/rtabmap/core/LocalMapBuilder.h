#pragma once

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SemanticDilation.h>
#include <rtabmap/core/RayTracing.h>
#include <rtabmap/core/LocalMap.h>
#include <rtabmap/core/Color.h>
#include <rtabmap/core/Grid.h>

#include <yaml-cpp/yaml.h>

#include <vector>
#include <utility>
#include <Eigen/Core>

namespace rtabmap {

template <int Dims>
class LocalMapBuilder
{
public:
    struct Area
    {
        float length = 0.0f;
        float width = 0.0f;
        float height = 0.0f;
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float roll = 0.0f;
        float pitch = 0.0f;
        float yaw = 0.0f;
        bool transparent = false;

        static Area createArea(const YAML::Node& node)
        {
            UASSERT(node.IsMap());
            UASSERT(node["length"]);
            UASSERT(node["width"]);
            UASSERT(node["height"]);
            UASSERT(node["x"]);
            UASSERT(node["y"]);
            UASSERT(node["z"]);
            UASSERT(node["roll"]);
            UASSERT(node["pitch"]);
            UASSERT(node["yaw"]);

            Area area;
            area.length = node["length"].as<float>();
            area.width = node["width"].as<float>();
            area.height = node["height"].as<float>();
            area.x = node["x"].as<float>();
            area.y = node["y"].as<float>();
            area.z = node["z"].as<float>();
            area.roll = node["roll"].as<float>();
            area.pitch = node["pitch"].as<float>();
            area.yaw = node["yaw"].as<float>();
            if (node["transparent"])
            {
                area.transparent = node["transparent"].as<bool>();
            }

            return area;
        }
    };

    struct Parameters
    {
        float cellSize = 0.1f;
        float maxVisibleRange = -1.0f;  // inf
        std::vector<Area> sensorIgnoreAreas;
        float minObstacleHeight = 0.2f;
        float maxObstacleHeight = 1.5f;
        float minSemanticRange = 0.0f;
        float maxSemanticRange = -1.0f;  // inf
        float sensorBlindRange2d = -1.0f;  // disabled
        bool enableRayTracing = false;
        float maxRange2d = 10.0f;  // (-1) - inf

        SemanticDilation::Parameters semanticDilationParameters;
        typename RayTracing<Dims>::Parameters rayTracingParameters;

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
            if (node["SensorIgnoreAreas"])
            {
                UASSERT(node["SensorIgnoreAreas"].IsSequence());
                for (const YAML::Node& areaNode : node["SensorIgnoreAreas"])
                {
                    parameters.sensorIgnoreAreas.push_back(Area::createArea(areaNode));
                }
            }
            if (node["MinObstacleHeight"])
            {
                parameters.minObstacleHeight = node["MinObstacleHeight"].as<float>();
            }
            if (node["MaxObstacleHeight"])
            {
                parameters.maxObstacleHeight = node["MaxObstacleHeight"].as<float>();
            }
            if (node["MinSemanticRange"])
            {
                parameters.minSemanticRange = node["MinSemanticRange"].as<float>();
            }
            if (node["MaxSemanticRange"])
            {
                parameters.maxSemanticRange = node["MaxSemanticRange"].as<float>();
            }
            if (node["SensorBlindRange2d"])
            {
                parameters.sensorBlindRange2d = node["SensorBlindRange2d"].as<float>();
            }
            if (node["EnableRayTracing"])
            {
                parameters.enableRayTracing = node["EnableRayTracing"].as<bool>();
            }
            if (node["MaxRange2d"])
            {
                parameters.maxRange2d = node["MaxRange2d"].as<float>();
            }
            if (node["SemanticDilation"])
            {
                parameters.semanticDilationParameters =
                    SemanticDilation::Parameters::createParameters(
                        node["SemanticDilation"]);
            }
            if (node["RayTracing"])
            {
                parameters.rayTracingParameters =
                    RayTracing<Dims>::Parameters::createParameters(
                        node["RayTracing"]);
            }
            return parameters;
        }
    };

    static const cv::Vec3b semanticBackgroundColor;  // (0, 0, 0)

public:
    LocalMapBuilder(const Parameters& parameters);
    void parseParameters(const Parameters& parameters);

    std::shared_ptr<LocalMap<Dims>> createLocalMap(const SensorData& sensorData,
        const Time& time, const Transform& fromUpdatedPose) const;

    const cv::Mat& lastDilatedSemantic() const { return lastDilatedSemantic_; }
    const std::vector<Area>& sensorIgnoreAreas() const { return sensorIgnoreAreas_; }

private:
    Eigen::Matrix3Xf filterMaxVisibleRange(const Eigen::Matrix3Xf& points) const;
    Eigen::Matrix3Xf transformPoints(const Eigen::Matrix3Xf& points,
        const Transform& transform) const;
    Eigen::Matrix3Xf getObstaclePoints(const Eigen::Matrix3Xf& points) const;
    std::pair<Eigen::Matrix3Xf, Eigen::Matrix3Xf> applySensorIgnoreAreas(
        const Eigen::Matrix3Xf& points, const Transform& toSensor) const;

    std::vector<Color> getPointsColors(const Eigen::Matrix3Xf& points,
        const std::vector<SensorData::CameraData>& camerasData) const;

    typename LocalMap<Dims>::ColoredGrid coloredGridFromObstacles(const Eigen::Matrix3Xf& points,
        const Eigen::Matrix3Xf& ignorePoints,
        const std::vector<Color>& colors,
        const std::array<float, Dims>& sensor) const;
    void traceRays(typename LocalMap<Dims>::ColoredGrid& coloredGrid,
        const std::array<float, Dims>& sensor) const;

private:
    float cellSize_;
    float maxVisibleRange_;
    float maxVisibleRangeSqr_;
    std::vector<Area> sensorIgnoreAreas_;
    std::vector<Transform> fromSensorIgnoreAreas_;
    std::vector<std::pair<float, float>> sensorIgnoreAreaXRanges_;
    std::vector<std::pair<float, float>> sensorIgnoreAreaYRanges_;
    std::vector<std::pair<float, float>> sensorIgnoreAreaZRanges_;
    std::vector<std::int8_t> sensorIgnoreAreasTransparent_;  // to avoid std::vector<bool> specialization
    float minObstacleHeight_;
    float maxObstacleHeight_;
    float minSemanticRange_;
    float minSemanticRangeSqr_;
    float maxSemanticRange_;
    float maxSemanticRangeSqr_;
    bool enableRayTracing_;
    float maxRange2d_;
    float maxRange2dSqr_;

    std::unique_ptr<SemanticDilation> semanticDilation_;
    std::unique_ptr<RayTracing<Dims>> rayTracing_;

    mutable cv::Mat lastDilatedSemantic_;
};

typedef LocalMapBuilder<2> LocalMapBuilder2d;
typedef LocalMapBuilder<3> LocalMapBuilder3d;

}
