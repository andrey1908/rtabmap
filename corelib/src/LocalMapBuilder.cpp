#include <rtabmap/core/LocalMapBuilder.h>

#include <set>
#include <limits>

#include <kas_utils/time_measurer.h>

namespace rtabmap {

const cv::Vec3b LocalMapBuilder::semanticBackgroundColor(0, 0, 0);

LocalMapBuilder::LocalMapBuilder(const Parameters& parameters)
{
    parseParameters(parameters);
}

void LocalMapBuilder::parseParameters(const Parameters& parameters)
{
    cellSize_ = parameters.cellSize;
    maxVisibleRange_ = parameters.maxVisibleRange;
    sensorIgnoreAreas_ = parameters.sensorIgnoreAreas;
    minObstacleHeight_ = parameters.minObstacleHeight;
    maxObstacleHeight_ = parameters.maxObstacleHeight;
    minSemanticRange_ = parameters.minSemanticRange;
    maxSemanticRange_ = parameters.maxSemanticRange;
    enableRayTracing_ = parameters.enableRayTracing;
    maxRange2d_ = parameters.maxRange2d;
    UASSERT(cellSize_ > 0.0f);
    UASSERT(minObstacleHeight_ < maxObstacleHeight_);
    UASSERT(minSemanticRange_ >= 0.0f);
    UASSERT(maxSemanticRange_ < 0.0f || minSemanticRange_ < maxSemanticRange_);

    for (const Area& area : sensorIgnoreAreas_)
    {
        UASSERT(area.length >= 0.0f);
        UASSERT(area.width >= 0.0f);
        UASSERT(area.height >= 0.0f);

        Transform toSensorIgnoreArea(
            area.x, area.y, area.z,
            area.roll, area.pitch, area.yaw);
        Transform fromSensorIgnoreArea = toSensorIgnoreArea.inverse();
        fromSensorIgnoreAreas_.push_back(std::move(fromSensorIgnoreArea));

        std::pair<float, float> xRange = std::make_pair(-area.length / 2, area.length / 2);
        std::pair<float, float> yRange = std::make_pair(-area.width / 2, area.width / 2);
        std::pair<float, float> zRange = std::make_pair(-area.height / 2, area.height / 2);
        sensorIgnoreAreaXRanges_.push_back(xRange);
        sensorIgnoreAreaYRanges_.push_back(yRange);
        sensorIgnoreAreaZRanges_.push_back(zRange);

        sensorIgnoreAreasTransparent_.push_back(area.transparent);
    }

    if (maxVisibleRange_ >= 0.0f)
    {
        maxVisibleRangeSqr_ = maxVisibleRange_ * maxVisibleRange_;
    }
    else
    {
        maxVisibleRangeSqr_ = -1.0f;
    }
    minSemanticRangeSqr_ = minSemanticRange_ * minSemanticRange_;
    if (maxSemanticRange_ >= 0.0f)
    {
        maxSemanticRangeSqr_ = maxSemanticRange_ * maxSemanticRange_;
    }
    else
    {
        maxSemanticRangeSqr_ = -1.0f;
    }
    if (maxRange2d_ >= 0.0f)
    {
        maxRange2dSqr_ = maxRange2d_ * maxRange2d_;
    }
    else
    {
        maxRange2dSqr_ = -1.0f;
    }

    semanticDilation_ = std::make_unique<SemanticDilation>(
        parameters.semanticDilationParameters);
    if (enableRayTracing_)
    {
        RayTracing::Parameters rayTracingParameters = parameters.rayTracingParameters;
        rayTracingParameters.cellSize = parameters.cellSize;
        rayTracingParameters.sensorBlindRange2d = parameters.sensorBlindRange2d;
        rayTracing_ = std::make_unique<RayTracing>(rayTracingParameters);
    }
    else
    {
        rayTracing_.reset();
    }
}

std::shared_ptr<LocalMap> LocalMapBuilder::createLocalMap(
    const SensorData& sensorData, const Time& time,
    const Transform& fromUpdatedPose) const
{
    MEASURE_BLOCK_TIME(LocalMapBuilder__createLocalMap);
    UASSERT(sensorData.numLidarsData() == 1);
    UASSERT(!fromUpdatedPose.isNull());
    const SensorData::LidarData& lidarData = sensorData.lidarsData().front();
    const Transform& toSensor = lidarData.toSensor;

    Eigen::Matrix3Xf points;
    if (maxVisibleRangeSqr_ >= 0.0f)
    {
        points = filterMaxVisibleRange(lidarData.points);
        points = transformPoints(points, toSensor);
    }
    else
    {
        points = transformPoints(lidarData.points, toSensor);
    }

    Eigen::Matrix3Xf obstacles = getObstaclePoints(points);
    Eigen::Matrix3Xf ignoreObstacles;
    if (fromSensorIgnoreAreas_.size())
    {
        std::tie(obstacles, ignoreObstacles) = applySensorIgnoreAreas(obstacles, toSensor);
    }

    std::vector<Color> colors;
    if (sensorData.numCamerasData())
    {
        if (semanticDilation_->dilationSize() > 0)
        {
            std::vector<SensorData::CameraData> dilatedCamerasData;
            for (const SensorData::CameraData& cameraData : sensorData.camerasData())
            {
                MEASURE_BLOCK_TIME(LocalMapBuilder__dilate);
                const cv::Mat& image = cameraData.image;
                cv::Mat dilatedImage =
                    semanticDilation_->dilate(image, {semanticBackgroundColor});
                SensorData::CameraData dilatedCameraData{
                    cameraData.toSensor, cameraData.parameters, dilatedImage};
                dilatedCamerasData.push_back(std::move(dilatedCameraData));
                lastDilatedSemantic_ = dilatedImage;
            }
            colors = getPointsColors(obstacles, dilatedCamerasData);
        }
        else
        {
            colors = getPointsColors(obstacles, sensorData.camerasData());
        }
    }
    else
    {
        colors.resize(obstacles.cols(), Color::missingColor);
    }

    Eigen::Vector2f sensor;
    sensor.x() = toSensor.translation().x();
    sensor.y() = toSensor.translation().y();
    LocalMap::ColoredGrid coloredGrid =
        coloredGridFromObstacles(obstacles, ignoreObstacles, colors, sensor);
    if (enableRayTracing_)
    {
        traceRays(coloredGrid, sensor);
    }

    auto localMap = std::make_shared<LocalMap>(
        coloredGrid, maxRange2dSqr_, true /* duplicatePoints */);
    localMap->setFromUpdatedPose(fromUpdatedPose);
    localMap->setTime(time);
    return localMap;
}

Eigen::Matrix3Xf LocalMapBuilder::filterMaxVisibleRange(const Eigen::Matrix3Xf& points) const
{
    std::vector<int> indices;
    indices.reserve(points.cols());
    for (int i = 0; i < points.cols(); i++)
    {
        float x = points(0, i);
        float y = points(1, i);
        float z = points(2, i);
        if (x * x + y * y + z * z <= maxVisibleRangeSqr_)
        {
            indices.push_back(i);
        }
    }

    Eigen::Matrix3Xf filtered(3, indices.size());
    int i = 0;
    for (int index : indices)
    {
        filtered(0, i) = points(0, index);
        filtered(1, i) = points(1, index);
        filtered(2, i) = points(2, index);
        i++;
    }
    return filtered;
}

Eigen::Matrix3Xf LocalMapBuilder::transformPoints(const Eigen::Matrix3Xf& points,
    const Transform& transform) const
{
    Eigen::Matrix3Xf transformed =
        (transform.toEigen3fRotation() * points).colwise() +
        transform.toEigen3fTranslation();
    return transformed;
}

Eigen::Matrix3Xf LocalMapBuilder::getObstaclePoints(const Eigen::Matrix3Xf& points) const
{
    std::vector<int> obstaclePointsIndices;
    obstaclePointsIndices.reserve(points.cols());
    for (int i = 0; i < points.cols(); i++)
    {
        float z = points(2, i);
        if (minObstacleHeight_ <= z && z <= maxObstacleHeight_)
        {
            obstaclePointsIndices.push_back(i);
        }
    }

    Eigen::Matrix3Xf obstaclePoints(3, obstaclePointsIndices.size());
    int i = 0;
    for (int index : obstaclePointsIndices)
    {
        obstaclePoints(0, i) = points(0, index);
        obstaclePoints(1, i) = points(1, index);
        obstaclePoints(2, i) = points(2, index);
        i++;
    }
    return obstaclePoints;
}

std::pair<Eigen::Matrix3Xf, Eigen::Matrix3Xf> LocalMapBuilder::applySensorIgnoreAreas(
    const Eigen::Matrix3Xf& points, const Transform& toSensor) const
{
    MEASURE_BLOCK_TIME(LocalMapBuilder__applySensorIgnoreAreas);
    std::set<int> indicesToIgnore;
    std::set<int> indicesToRemove;
    Transform fromSensor = toSensor.inverse();
    for (int i = 0; i < fromSensorIgnoreAreas_.size(); i++)
    {
        const Transform& fromSensorIgnoreArea = fromSensorIgnoreAreas_[i];
        const std::pair<float, float>& xRange = sensorIgnoreAreaXRanges_[i];
        const std::pair<float, float>& yRange = sensorIgnoreAreaYRanges_[i];
        const std::pair<float, float>& zRange = sensorIgnoreAreaZRanges_[i];
        bool transparent = sensorIgnoreAreasTransparent_[i];

        Eigen::Matrix3Xf pointsInArea = transformPoints(points, fromSensorIgnoreArea * fromSensor);
        for (int j = 0; j < pointsInArea.cols(); j++)
        {
            float x = pointsInArea(0, j);
            float y = pointsInArea(1, j);
            float z = pointsInArea(2, j);
            if (xRange.first <= x && x <= xRange.second &&
                yRange.first <= y && y <= yRange.second &&
                zRange.first <= z && z <= zRange.second)
            {
                if (!transparent)
                {
                    indicesToIgnore.insert(j);
                }
                else
                {
                    indicesToRemove.insert(j);
                }
            }
        }
    }

    int numUniqueIndices = indicesToIgnore.size() + indicesToRemove.size();
    auto indicesToIgnoreIt = indicesToIgnore.cbegin();
    auto indicesToRemoveIt = indicesToRemove.cbegin();
    auto indicesToIgnoreEnd = indicesToIgnore.cend();
    auto indicesToRemoveEnd = indicesToRemove.cend();
    while (indicesToIgnoreIt != indicesToIgnoreEnd && indicesToRemoveIt != indicesToRemoveEnd)
    {
        int indexToIgnore = *indicesToIgnoreIt;
        int indexToRemove = *indicesToRemoveIt;
        if (indexToIgnore < indexToRemove)
        {
            ++indicesToIgnoreIt;
        }
        else if (indexToIgnore > indexToRemove)
        {
            ++indicesToRemoveIt;
        }
        else
        {
            ++indicesToIgnoreIt;
            ++indicesToRemoveIt;
            numUniqueIndices--;
        }
    }

    Eigen::Matrix3Xf keepPoints(3, points.cols() - numUniqueIndices);
    Eigen::Matrix3Xf ignorePoints(3, indicesToIgnore.size());
    indicesToIgnore.insert(indicesToIgnore.end(), std::numeric_limits<int>::max());
    indicesToRemove.insert(indicesToRemove.end(), std::numeric_limits<int>::max());
    indicesToIgnoreIt = indicesToIgnore.cbegin();
    indicesToRemoveIt = indicesToRemove.cbegin();
    int keepIndex = 0;
    int ignoreIndex = 0;
    for (int index = 0; index < points.cols(); index++)
    {
        bool ignorePoint = (index == *indicesToIgnoreIt);
        bool removePoint = (index == *indicesToRemoveIt);

        if (!ignorePoint && !removePoint)
        {
            keepPoints(0, keepIndex) = points(0, index);
            keepPoints(1, keepIndex) = points(1, index);
            keepPoints(2, keepIndex) = points(2, index);
            keepIndex++;
            continue;
        }
        if (ignorePoint)
        {
            ignorePoints(0, ignoreIndex) = points(0, index);
            ignorePoints(1, ignoreIndex) = points(1, index);
            ignorePoints(2, ignoreIndex) = points(2, index);
            ignoreIndex++;
            ++indicesToIgnoreIt;
        }
        if (removePoint)
        {
            ++indicesToRemoveIt;
        }
    }
    UASSERT(keepIndex == keepPoints.cols());
    UASSERT(ignoreIndex == ignorePoints.cols());
    UASSERT(indicesToIgnoreIt == std::prev(indicesToIgnore.cend()));
    UASSERT(indicesToRemoveIt == std::prev(indicesToRemove.cend()));

    return std::make_pair(std::move(keepPoints), std::move(ignorePoints));
}

std::vector<Color> LocalMapBuilder::getPointsColors(
    const Eigen::Matrix3Xf& points,
    const std::vector<SensorData::CameraData>& camerasData) const
{
    std::vector<Color> colors;
    colors.resize(points.cols(), Color::missingColor);
    for (const SensorData::CameraData& cameraData : camerasData)
    {
        const Transform& toSensor = cameraData.toSensor;
        const SensorData::CameraParameters& parameters = cameraData.parameters;
        const cv::Mat& image = cameraData.image;
        UASSERT(image.type() == CV_8UC3);

        Transform fromSensor = toSensor.inverse();
        const Eigen::Matrix3Xf& transformedPoints =
            (fromSensor.toEigen3fRotation() * points).colwise() +
            fromSensor.toEigen3fTranslation();
        for (int i = 0; i < transformedPoints.cols(); i++)
        {
            float x = transformedPoints(0, i);
            float y = transformedPoints(1, i);
            float z = transformedPoints(2, i);
            SensorData::Pixel pixel = parameters.project(x, y, z);
            if (parameters.inFrame(pixel))
            {
                float rangeSqr = x * x + y * y + z * z;
                if (rangeSqr >= minSemanticRangeSqr_ &&
                    (maxSemanticRangeSqr_ < 0.0f || rangeSqr <= maxSemanticRangeSqr_))
                {
                    const cv::Vec3b& pixelColor = image.at<cv::Vec3b>(pixel.v, pixel.u);
                    if (pixelColor == semanticBackgroundColor)
                    {
                        continue;
                    }
                    colors[i].b() = pixelColor[0];
                    colors[i].g() = pixelColor[1];
                    colors[i].r() = pixelColor[2];
                    colors[i].missing() = false;
                }
            }
        }
    }
    return colors;
}

LocalMap::ColoredGrid LocalMapBuilder::coloredGridFromObstacles(
    const Eigen::Matrix3Xf& points,
    const Eigen::Matrix3Xf& ignorePoints,
    const std::vector<Color>& colors,
    const Eigen::Vector2f& sensor) const
{
    MapLimitsF limitsF;
    limitsF.update({sensor.y(), sensor.x()});
    for (int i = 0; i < points.cols(); i++)
    {
        float x = points(0, i);
        float y = points(1, i);
        limitsF.update({y, x});
    }
    if (enableRayTracing_ && rayTracing_->traceIntoUnknownSpace())
    {
        float range = rayTracing_->maxTracingRange();
        limitsF.update({sensor.y() - range, sensor.x() - range});
        limitsF.update({sensor.y() + range, sensor.x() + range});
    }

    LocalMap::ColoredGrid coloredGrid;
    coloredGrid.cellSize = cellSize_;
    coloredGrid.limits.set(
        {std::floor(limitsF.min()[0] / cellSize_), std::floor(limitsF.min()[1] / cellSize_)},
        {std::floor(limitsF.max()[0] / cellSize_), std::floor(limitsF.max()[1] / cellSize_)});
    int height = coloredGrid.limits.shape()[0];
    int width = coloredGrid.limits.shape()[1];
    coloredGrid.grid = cv::Mat(height, width, CV_8U, LocalMap::ColoredGrid::unknownCellValue);
    coloredGrid.colors = cv::Mat(height, width, CV_32S, Color::missingColor.data());
    const int& minX = coloredGrid.limits.min()[1];
    const int& minY = coloredGrid.limits.min()[0];

    for (int i = 0; i < ignorePoints.cols(); i++)
    {
        float xf = ignorePoints(0, i);
        float yf = ignorePoints(1, i);
        int y = std::floor(yf / cellSize_) - minY;
        int x = std::floor(xf / cellSize_) - minX;
        if (x < 0 || y < 0 || x >= width || y >= height)
        {
            continue;
        }
        coloredGrid.grid.at<std::uint8_t>(y, x) = LocalMap::ColoredGrid::ignoredOccupiedCellValue;
    }

    for (int i = 0; i < points.cols(); i++)
    {
        float xf = points(0, i);
        float yf = points(1, i);
        int y = std::floor(yf / cellSize_) - minY;
        int x = std::floor(xf / cellSize_) - minX;
        coloredGrid.grid.at<std::uint8_t>(y, x) = LocalMap::ColoredGrid::occupiedCellValue;
        const Color& pointColor = colors[i];
        if (pointColor != Color::missingColor)
        {
            Color& cellColor =
                reinterpret_cast<Color&>(coloredGrid.colors.at<std::int32_t>(y, x));
            if (pointColor.brightness() > cellColor.brightness())
            {
                cellColor = pointColor;
            }
        }
    }

    return coloredGrid;
}

void LocalMapBuilder::traceRays(LocalMap::ColoredGrid& coloredGrid,
    const Eigen::Vector2f& sensor) const
{
    MEASURE_BLOCK_TIME(LocalMapBuilder__traceRays);
    RayTracing::Cell origin;
    origin.y = std::floor(sensor.y() / cellSize_) - coloredGrid.limits.min()[0];
    origin.x = std::floor(sensor.x() / cellSize_) - coloredGrid.limits.min()[1];
    rayTracing_->traceRays(coloredGrid.grid, origin);
}

}
