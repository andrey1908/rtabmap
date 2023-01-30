#pragma once

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SemanticDilation.h>
#include <rtabmap/core/RayTracing.h>
#include <rtabmap/core/LocalMap.h>
#include <rtabmap/core/Color.h>
#include <rtabmap/core/Grid.h>

#include <vector>
#include <utility>
#include <Eigen/Core>

namespace rtabmap {

class LocalMapBuilder
{
public:
    static const cv::Vec3b semanticBackgroundColor;  // (0, 0, 0)

public:
    LocalMapBuilder(const ParametersMap& parameters = ParametersMap());
    void parseParameters(const ParametersMap& parameters);

    std::shared_ptr<LocalMap> createLocalMap(const Signature& signature) const;

    const cv::Mat& lastDilatedSemantic() const { return lastDilatedSemantic_; }

private:
    Eigen::Matrix3Xf convertLaserScan(const LaserScan& laserScan) const;
    Eigen::Matrix3Xf filterMaxVisibleRange(const Eigen::Matrix3Xf& points) const;
    Eigen::Matrix3Xf transformPoints(const Eigen::Matrix3Xf& points,
        const Transform& transform) const;
    Eigen::Matrix3Xf getObstaclePoints(const Eigen::Matrix3Xf& points) const;

    std::vector<Color> getPointsColors(const Eigen::Matrix3Xf& points,
        const std::vector<cv::Mat>& images,
        const std::vector<rtabmap::CameraModel>& cameraModels) const;

    LocalMap::ColoredGrid coloredGridFromObstacles(const Eigen::Matrix3Xf& points,
        const std::vector<Color>& colors,
        const Eigen::Vector2f& sensor) const;
    void traceRays(LocalMap::ColoredGrid& coloredGrid,
        const Eigen::Vector2f& sensor) const;

private:
    float cellSize_;
    float maxVisibleRange_;
    float maxVisibleRangeSqr_;
    float maxRange2d_;
    float maxRange2dSqr_;
    float minObstacleHeight_;
    float maxObstacleHeight_;
    float minSemanticRange_;
    float minSemanticRangeSqr_;
    float maxSemanticRange_;
    float maxSemanticRangeSqr_;
    bool enableRayTracing_;
    float sensorBlindRange2d_;
    float sensorBlindRange2dSqr_;

    std::unique_ptr<SemanticDilation> semanticDilation_;
    std::unique_ptr<RayTracing> rayTracing_;

    mutable cv::Mat lastDilatedSemantic_;
};

}
