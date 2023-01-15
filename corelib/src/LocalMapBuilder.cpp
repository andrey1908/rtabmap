#include <rtabmap/core/LocalMapBuilder.h>
#include <rtabmap/utilite/ULogger.h>

#include <limits>

#include "time_measurer/time_measurer.h"

namespace rtabmap {

const cv::Vec3b LocalMapBuilder::semanticBackgroundColor(0, 0, 0);

LocalMapBuilder::LocalMapBuilder(const ParametersMap& parameters) :
    cellSize_(Parameters::defaultGridCellSize()),
    maxVisibleRange_(Parameters::defaultLocalMapMaxVisibleRange()),
    maxRange2d_(Parameters::defaultLocalMapMaxRange2d()),
    minObstacleHeight_(Parameters::defaultLocalMapMinObstacleHeight()),
    maxObstacleHeight_(Parameters::defaultLocalMapMaxObstacleHeight()),
    minSemanticRange_(Parameters::defaultLocalMapMinSemanticRange()),
    maxSemanticRange_(Parameters::defaultLocalMapMaxSemanticRange()),
    useRayTracing_(Parameters::defaultLocalMapUseRayTracing()),
    sensorBlindRange2d_(Parameters::defaultLocalMapSensorBlindRange2d())
{
    parseParameters(parameters);
}

void LocalMapBuilder::parseParameters(const ParametersMap& parameters)
{
    Parameters::parse(parameters, Parameters::kGridCellSize(), cellSize_);
    Parameters::parse(parameters, Parameters::kLocalMapMaxVisibleRange(), maxVisibleRange_);
    Parameters::parse(parameters, Parameters::kLocalMapMaxRange2d(), maxRange2d_);
    Parameters::parse(parameters, Parameters::kLocalMapMinObstacleHeight(), minObstacleHeight_);
    Parameters::parse(parameters, Parameters::kLocalMapMaxObstacleHeight(), maxObstacleHeight_);
    Parameters::parse(parameters, Parameters::kLocalMapMinSemanticRange(), minSemanticRange_);
    Parameters::parse(parameters, Parameters::kLocalMapMaxSemanticRange(), maxSemanticRange_);
    Parameters::parse(parameters, Parameters::kLocalMapUseRayTracing(), useRayTracing_);
    Parameters::parse(parameters, Parameters::kLocalMapSensorBlindRange2d(), sensorBlindRange2d_);
    UASSERT(minObstacleHeight_ < maxObstacleHeight_);
    UASSERT(maxSemanticRange_ == 0.0f || minSemanticRange_ < maxSemanticRange_);

    maxVisibleRangeSqr_ = maxVisibleRange_ * maxVisibleRange_;
    maxRange2dSqr_ = maxRange2d_ * maxRange2d_;
    minSemanticRangeSqr_ = minSemanticRange_ * minSemanticRange_;
    maxSemanticRangeSqr_ = maxSemanticRange_ * maxSemanticRange_;
    sensorBlindRange2dSqr_ = sensorBlindRange2d_ * sensorBlindRange2d_;

    semanticDilation_ = std::make_unique<SemanticDilation>(parameters);
    if (useRayTracing_)
    {
        rayTracing_ = std::make_unique<RayTracing>(parameters);
    }
    else
    {
        rayTracing_.reset();
    }
}

std::shared_ptr<LocalMap> LocalMapBuilder::createLocalMap(
    const Signature& signature) const
{
    MEASURE_BLOCK_TIME(LocalMapBuilder__createLocalMap);
    UASSERT(!signature.sensorData().laserScan().isEmpty() &&
            !signature.sensorData().laserScan().is2d());
    const LaserScan& laserScan = signature.sensorData().laserScan();
    const Transform& transform = laserScan.localTransform();
    Eigen::Matrix3Xf points, obstacles;
    points = convertLaserScan(laserScan);
    if (maxVisibleRangeSqr_ != 0.0f)
    {
        points = filterMaxVisibleRange(points);
    }
    points = transformPoints(points, transform);
    obstacles = getObstaclePoints(points);

    std::vector<Color> colors;
    if (signature.sensorData().images().size())
    {
        if (semanticDilation_->dilationSize() > 0)
        {
            std::vector<cv::Mat> dilatedImages;
            for (const cv::Mat& image : signature.sensorData().images())
            {
                MEASURE_BLOCK_TIME(LocalMapBuilder__dilate);
                cv::Mat dilatedImage =
                    semanticDilation_->dilate(image, {semanticBackgroundColor});
                dilatedImages.push_back(dilatedImage);
                lastDilatedSemantic_ = dilatedImage;
            }
            colors = getPointsColors(obstacles, dilatedImages,
                signature.sensorData().cameraModels());
        }
        else
        {
            colors = getPointsColors(obstacles, signature.sensorData().images(),
                signature.sensorData().cameraModels());
        }
    }
    else
    {
        colors.resize(obstacles.cols(), Color::missingColor);
    }

    ColoredGrid coloredGrid;
    Eigen::Vector2f sensor;
    sensor.x() = transform.translation().x();
    sensor.y() = transform.translation().y();
    coloredGrid = coloredGridFromObstacles(obstacles, colors, sensor);
    if (useRayTracing_)
    {
        traceRays(coloredGrid, sensor);
    }
    
    std::shared_ptr<LocalMap> localMap = localMapFromColoredGrid(coloredGrid);

    localMap->sensorBlindRange2dSqr = sensorBlindRange2dSqr_;
    localMap->toSensor = signature.sensorData().laserScan().localTransform();
    UASSERT(!localMap->toSensor.isNull());
    localMap->fromUpdatedPose = Transform::getIdentity();
    return localMap;
}

Eigen::Matrix3Xf LocalMapBuilder::convertLaserScan(const LaserScan& laserScan) const
{
    Eigen::Matrix3Xf points(3, laserScan.size());
    for (int i = 0; i < laserScan.size(); i++)
    {
        float x = laserScan.data().ptr<float>(0, i)[0];
        float y = laserScan.data().ptr<float>(0, i)[1];
        float z = laserScan.data().ptr<float>(0, i)[2];
        points(0, i) = x;
        points(1, i) = y;
        points(2, i) = z;
    }
    return points;
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

std::vector<Color> LocalMapBuilder::getPointsColors(
    const Eigen::Matrix3Xf& points,
    const std::vector<cv::Mat>& images,
    const std::vector<CameraModel>& cameraModels) const
{
    UASSERT(images.size() == cameraModels.size());
    std::vector<Color> colors;
    colors.resize(points.cols(), Color::missingColor);
    for (int camI = 0; camI < images.size(); camI++)
    {
        const cv::Mat& image = images[camI];
        UASSERT(image.type() == CV_8UC3);
        const CameraModel& cameraModel = cameraModels[camI];
        const Transform& transform = cameraModel.localTransform().inverse();
        const Eigen::Matrix3Xf& transformedPoints =
            (transform.toEigen3fRotation() * points).colwise() +
            transform.toEigen3fTranslation();
        for (int i = 0; i < transformedPoints.cols(); i++)
        {
            float x = transformedPoints(0, i);
            float y = transformedPoints(1, i);
            float z = transformedPoints(2, i);
            float rangeSqr = x * x + y * y + z * z;
            int u, v;
            cameraModel.reproject(x, y, z, u, v);
            if (cameraModel.inFrame(u, v) && z > 0 &&
                rangeSqr >= minSemanticRangeSqr_ &&
                (maxSemanticRangeSqr_ == 0.0f || rangeSqr <= maxSemanticRangeSqr_))
            {
                const cv::Vec3b& pixelColor = image.at<cv::Vec3b>(v, u);
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
    return colors;
}

LocalMapBuilder::ColoredGrid LocalMapBuilder::coloredGridFromObstacles(
    const Eigen::Matrix3Xf& points,
    const std::vector<Color>& colors,
    const Eigen::Vector2f& sensor) const
{
    float minXf = sensor.x();
    float minYf = sensor.y();
    float maxXf = sensor.x();
    float maxYf = sensor.y();
    for (int i = 0; i < points.cols(); i++)
    {
        float x = points(0, i);
        float y = points(1, i);
        minXf = std::min(minXf, x);
        minYf = std::min(minYf, y);
        maxXf = std::max(maxXf, x);
        maxYf = std::max(maxYf, y);
    }
    if (useRayTracing_ && rayTracing_->traceRaysIntoUnknownSpace())
    {
        float range = rayTracing_->maxRayTracingRange();
        minXf = std::min(minXf, -range + sensor.x());
        minYf = std::min(minYf, -range + sensor.y());
        maxXf = std::max(maxXf,  range + sensor.x());
        maxYf = std::max(maxYf,  range + sensor.y());
    }

    ColoredGrid coloredGrid;
    coloredGrid.minY = std::floor(minYf / cellSize_);
    coloredGrid.minX = std::floor(minXf / cellSize_);
    int maxY = std::floor(maxYf / cellSize_);
    int maxX = std::floor(maxXf / cellSize_);
    int height = maxY - coloredGrid.minY + 1;
    int width = maxX - coloredGrid.minX + 1;
    coloredGrid.grid = cv::Mat(height, width, CV_8S, RayTracing::unknownCellValue);
    coloredGrid.colors = cv::Mat(height, width, CV_32S, Color::missingColor.data());
    for (int i = 0; i < points.cols(); i++)
    {
        float yf = points(1, i);
        float xf = points(0, i);
        int y = std::floor(yf / cellSize_) - coloredGrid.minY;
        int x = std::floor(xf / cellSize_) - coloredGrid.minX;
        coloredGrid.grid.at<std::int8_t>(y, x) = RayTracing::occupiedCellValue;
        const Color& pointColor = colors[i];
        if (pointColor != Color::missingColor)
        {
            Color& cellColor =
                reinterpret_cast<Color&>(coloredGrid.colors.at<int>(y, x));
            if (pointColor.brightness() > cellColor.brightness())
            {
                cellColor = pointColor;
            }
        }
    }
    return coloredGrid;
}

void LocalMapBuilder::traceRays(ColoredGrid& coloredGrid,
    const Eigen::Vector2f& sensor) const
{
    MEASURE_BLOCK_TIME(LocalMapBuilder__traceRays);
    RayTracing::Cell origin;
    origin.y = std::floor(sensor.y() / cellSize_) - coloredGrid.minY;
    origin.x = std::floor(sensor.x() / cellSize_) - coloredGrid.minX;
    rayTracing_->traceRays(coloredGrid.grid, origin);
}

std::shared_ptr<LocalMap> LocalMapBuilder::localMapFromColoredGrid(
    const ColoredGrid& coloredGrid) const
{
    std::vector<std::pair<int, int>> occupiedCells;
    std::vector<std::pair<int, int>> emptyCells;
    for (int y = 0; y < coloredGrid.grid.rows; y++)
    {
        for (int x = 0; x < coloredGrid.grid.cols; x++)
        {
            if (maxRange2dSqr_ != 0.0f)
            {
                float xf = (x + coloredGrid.minX + 0.5f) * cellSize_;
                float yf = (y + coloredGrid.minY + 0.5f) * cellSize_;
                if (xf * xf + yf * yf > maxRange2dSqr_)
                {
                    continue;
                }
            }
            std::int8_t value = coloredGrid.grid.at<std::int8_t>(y, x);
            if (value == RayTracing::occupiedCellValue)
            {
                occupiedCells.emplace_back(y, x);
            }
            else if (value == RayTracing::emptyCellValue)
            {
                emptyCells.emplace_back(y, x);
            }
        }
    }

    std::shared_ptr<LocalMap> localMap = std::make_shared<LocalMap>();
    localMap->numObstacles = occupiedCells.size() * 2;
    localMap->numEmpty = emptyCells.size() * 2;
    localMap->points.resize(3, localMap->numObstacles + localMap->numEmpty);
    localMap->colors.reserve(localMap->numObstacles + localMap->numEmpty);
    for (int i = 0, cellI = 0; i < localMap->numObstacles + localMap->numEmpty;
        i += 2, cellI++)
    {
        int x;
        int y;
        if (cellI < occupiedCells.size())
        {
            const std::pair<int, int>& occupiedCell = occupiedCells[cellI];
            x = occupiedCell.second;
            y = occupiedCell.first;
        }
        else
        {
            const std::pair<int, int>& emptyCell = emptyCells[cellI - occupiedCells.size()];
            x = emptyCell.second;
            y = emptyCell.first;
        }
        float xf = (x + coloredGrid.minX + 0.5f) * cellSize_;
        float yf = (y + coloredGrid.minY + 0.5f) * cellSize_;
        const Color& color =
            reinterpret_cast<const Color&>(coloredGrid.colors.at<int>(y, x));
        // this format with duplicated points is assumed in other code
        localMap->points(0, i) = xf;
        localMap->points(1, i) = yf;
        localMap->points(2, i) = 0.0f;
        localMap->colors.push_back(color);
        localMap->points(0, i + 1) = xf + cellSize_ * 0.5f;
        localMap->points(1, i + 1) = yf + cellSize_ * 0.5f;
        localMap->points(2, i + 1) = 0.0f;
        localMap->colors.push_back(color);
    }
    return localMap;
}

}
