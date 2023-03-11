#include <rtabmap/core/ObjectTracking.h>

#include <deque>
#include <algorithm>

#include "time_measurer/time_measurer.h"

namespace rtabmap {

ObjectTracking::ObjectTracking(float cellSize) :
    cellSize_(cellSize)
{
    constexpr int size = 1;
    for (int y = -size; y <= size; y++)
    {
        for (int x = -size; x <= size; x++)
        {
            if (y != 0 || x != 0)
            {
                neighborCells_.emplace_back(y, x);
            }
        }
    }
};

void ObjectTracking::track(const LocalMap& localMap, const Transform& pose)
{
    MEASURE_BLOCK_TIME(ObjectTracking__track);
    float dt = localMap.time().toSec() - prevTime_.toSec();
    std::vector<TrackedObject> trackedObjects = detect(localMap, pose);
    std::vector<TrackedObject> assignedTrackedObjects = assign(trackedObjects, dt);
    update(trackedObjects, assignedTrackedObjects, dt);
    trackedObjects_ = std::move(trackedObjects);
    prevTime_ = localMap.time();
}

std::vector<ObjectTracking::TrackedObject> ObjectTracking::detect(
    const LocalMap& localMap, const Transform& pose)
{
    LocalMap::ColoredGrid coloredGrid = localMap.toColoredGrid();
    MapLimitsI mapLimits = coloredGrid.limits;
    cv::Mat colorGrid = coloredGrid.colors;
    std::vector<TrackedObject> trackedObjects;
    for (int y = 0; y < colorGrid.rows; y++)
    {
        for (int x = 0; x < colorGrid.cols; x++)
        {
            if (colorGrid.at<std::int32_t>(y, x) != Color::missingColor.data())
            {
                TrackedObject trackedObject = segment(colorGrid, Cell(y, x), mapLimits,
                    pose);
                if (trackedObject.object.size() >= 4)
                {
                    trackedObjects.push_back(std::move(trackedObject));
                }
            }
        }
    }
    return trackedObjects;
}

ObjectTracking::TrackedObject ObjectTracking::segment(
    cv::Mat& colorGrid, const Cell& startCell, const MapLimitsI& mapLimits,
    const Transform& pose)
{
    Cell shift(mapLimits.minY(), mapLimits.minX());

    std::int32_t& startCellColor = colorGrid.at<std::int32_t>(startCell.y, startCell.x);
    Color segmentColor = reinterpret_cast<const Color&>(startCellColor);
    UASSERT(segmentColor != Color::missingColor);

    TrackedObject trackedObject;
    std::deque<Cell> queue;
    queue.push_back(startCell);
    startCellColor = Color::missingColor.data();
    while (queue.size())
    {
        Cell currentCell = queue.front();
        queue.pop_front();
        trackedObject.object.push_back(currentCell + shift);

        for (const Cell& neighborCell : neighborCells_)
        {
            Cell nextCell = currentCell + neighborCell;
            int y = nextCell.y;
            int x = nextCell.x;
            std::int32_t& color = colorGrid.at<std::int32_t>(y, x);
            if (color == segmentColor.data())
            {
                queue.emplace_back(y, x);
                color = Color::missingColor.data();
            }
        }
    }

    float x = 0.0f;
    float y = 0.0f;
    float num = trackedObject.object.size();
    for (const Cell& cell : trackedObject.object)
    {
        x += cell.x / num;
        y += cell.y / num;
    }
    Eigen::Vector3f position;
    position.x() = x * cellSize_ + cellSize_ / 2;
    position.y() = y * cellSize_ + cellSize_ / 2;
    position.z() = 0.0f;
    position = pose.toEigen3fRotation() * position + pose.toEigen3fTranslation();
    trackedObject.position = Point(
        position.x(),
        position.y());
    trackedObject.color = segmentColor;

    return trackedObject;
}

std::vector<ObjectTracking::TrackedObject> ObjectTracking::assign(
    const std::vector<TrackedObject>& trackedObjects, float dt)
{
    std::vector<Score> scores;
    for (int oldIndex = 0; oldIndex < trackedObjects_.size(); oldIndex++)
    {
        const TrackedObject& oldTrackedObject = trackedObjects_[oldIndex];
        int oldSize = oldTrackedObject.object.size();
        const Point& oldPosition = oldTrackedObject.position;
        Point predictedPosition;
        predictedPosition.x = oldPosition.x + oldTrackedObject.velocity.vx * dt;
        predictedPosition.y = oldPosition.y + oldTrackedObject.velocity.vy * dt;
        float distanceFactor = 2.0f;
        if (oldTrackedObject.trackedTimes == 1)
        {
            distanceFactor = 0.4f;
        }
        for (int newIndex = 0; newIndex < trackedObjects.size(); newIndex++)
        {
            const TrackedObject& newTrackedObject = trackedObjects[newIndex];
            if (oldTrackedObject.color != newTrackedObject.color)
            {
                continue;
            }
            int newSize = newTrackedObject.object.size();
            const Point& newPosition = newTrackedObject.position;
            Score score;
            score.score =
                (1.0f * std::abs(oldSize - newSize) / std::min(oldSize, newSize)) +
                (Point::distanceSqr(predictedPosition, newPosition) * distanceFactor);
            score.oldIndex = oldIndex;
            score.newIndex = newIndex;
            scores.push_back(score);
        }
    }

    std::sort(scores.begin(), scores.end());

    std::vector<int> usedOldIndices;
    std::vector<int> usedNewIndices;
    std::vector<TrackedObject> assignedTrackedObjects(trackedObjects.size());
    for (const Score& score : scores)
    {
        if (score.score > 2.0f)
        {
            break;
        }
        int oldIndex = score.oldIndex;
        int newIndex = score.newIndex;
        bool usedOldIndex =
            std::find(usedOldIndices.begin(), usedOldIndices.end(), oldIndex) !=
                usedOldIndices.end();
        bool usedNewIndex =
            std::find(usedNewIndices.begin(), usedNewIndices.end(), newIndex) !=
                usedNewIndices.end();
        if (!usedOldIndex && !usedNewIndex)
        {
            assignedTrackedObjects[newIndex] = trackedObjects_[oldIndex];
            usedOldIndices.push_back(oldIndex);
            usedNewIndices.push_back(newIndex);
        }
    }

    return assignedTrackedObjects;
}

void ObjectTracking::update(std::vector<TrackedObject>& trackedObjects,
    const std::vector<TrackedObject>& assignedTrackedObjects, float dt)
{
    UASSERT(trackedObjects.size() == assignedTrackedObjects.size());
    for (int i = 0; i < trackedObjects.size(); i++)
    {
        TrackedObject& trackedObject = trackedObjects[i];
        const TrackedObject& assignedTrackedObject = assignedTrackedObjects[i];
        if (assignedTrackedObject.id != -1)
        {
            float dx = trackedObject.position.x - assignedTrackedObject.position.x;
            float dy = trackedObject.position.y - assignedTrackedObject.position.y;
            trackedObject.id = assignedTrackedObject.id;
            trackedObject.trackedTimes = assignedTrackedObject.trackedTimes + 1;
            trackedObject.velocity.vx = dx / dt;
            trackedObject.velocity.vy = dy / dt;

            float f = 0.5f;
            trackedObject.velocity.vx = f * trackedObject.velocity.vx +
                (1 - f) * assignedTrackedObject.velocity.vx;
            trackedObject.velocity.vy = f * trackedObject.velocity.vy +
                (1 - f) * assignedTrackedObject.velocity.vy;
        }
        else
        {
            trackedObject.id = nextTrackedId_;
            trackedObject.trackedTimes = 1;
            nextTrackedId_++;
        }
    }
}

}
