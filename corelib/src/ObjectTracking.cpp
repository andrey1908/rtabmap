#include <rtabmap/core/ObjectTracking.h>

#include <deque>
#include <algorithm>

#include "time_measurer/time_measurer.h"

namespace rtabmap {

void ObjectTracking::track(const LocalMap& localMap)
{
    MEASURE_BLOCK_TIME(_________track);
    std::vector<TrackedObject> trackedObjects = detect(localMap);
    std::vector<TrackedObject> assignedTrackedObjects = assign(trackedObjects);
    UASSERT(trackedObjects.size() == assignedTrackedObjects.size());
    for (int i = 0; i < trackedObjects.size(); i++)
    {
        TrackedObject& trackedObject = trackedObjects[i];
        const TrackedObject& assignedTrackedObject = assignedTrackedObjects[i];
        if (assignedTrackedObject.id != (std::int32_t)(-1))
        {
            float dx = trackedObject.position.x - assignedTrackedObject.position.x;
            float dy = trackedObject.position.y - assignedTrackedObject.position.y;
            float dt = 0.1;
            trackedObject.id = assignedTrackedObject.id;
            trackedObject.velocity.vx = dx / dt;
            trackedObject.velocity.vy = dy / dt;
        }
        else
        {
            trackedObject.id = nextTrackedId_;
            nextTrackedId_++;
        }
    }
    trackedObjects_ = std::move(trackedObjects);
}

std::vector<ObjectTracking::TrackedObject> ObjectTracking::detect(const LocalMap& localMap)
{
    LocalMap::ColoredGrid coloredGrid = localMap.toColoredGrid();
    MapLimitsI mapLimits = coloredGrid.limits;
    cv::Mat colorGrid = coloredGrid.colors;
    std::vector<TrackedObject> trackedObjects;
    for (int y = 0; y < colorGrid.rows; y++)
    {
        for (int x = 0; x < colorGrid.cols; x++)
        {
            if (colorGrid.at<std::int32_t>(y, x) == 9831741)
            {
                TrackedObject trackedObject = segment(colorGrid, Cell(y, x), mapLimits);
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
    cv::Mat& colorGrid, const Cell& startCell, const MapLimitsI& mapLimits)
{
    Cell shift(mapLimits.minY(), mapLimits.minX());
    TrackedObject trackedObject;
    std::deque<Cell> queue;
    queue.push_back(startCell);
    colorGrid.at<std::int32_t>(startCell.y, startCell.x) =
        Color::missingColor.data();
    while (queue.size())
    {
        Cell currentCell = queue.front();
        queue.pop_front();
        trackedObject.object.push_back(currentCell + shift);

        int y = currentCell.y;
        int x = currentCell.x;
        for (int dy = -1; dy <= 1; dy++)
        {
            for (int dx = -1; dx <= 1; dx++)
            {
                if (colorGrid.at<std::int32_t>(y + dy, x + dx) == 9831741)
                {
                    queue.emplace_back(y + dy, x + dx);
                    colorGrid.at<std::int32_t>(y + dy, x + dx) =
                        Color::missingColor.data();
                }
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
    trackedObject.position = Point(
        x * cellSize_ + cellSize_ / 2,
        y * cellSize_ + cellSize_ / 2);

    return trackedObject;
}

std::vector<ObjectTracking::TrackedObject> ObjectTracking::assign(
    const std::vector<TrackedObject>& trackedObjects)
{
    std::vector<Score> scores;
    for (int i = 0; i < trackedObjects_.size(); i++)
    {
        for (int j = 0; j < trackedObjects.size(); j++)
        {
            const TrackedObject& trackedObjectI = trackedObjects_[i];
            const TrackedObject& trackedObjectJ = trackedObjects[j];
            int sizeI = trackedObjectI.object.size();
            int sizeJ = trackedObjectJ.object.size();
            const Point& positionI = trackedObjectI.position;
            const Point& positionJ = trackedObjectJ.position;
            Score score;
            score.score = (1.0f * std::abs(sizeI - sizeJ) / std::min(sizeI, sizeJ)) +
                (Point::distanceSqr(positionI, positionJ) * 2.0f /* some factor */);
            score.i = i;
            score.j = j;
            scores.push_back(score);
        }
    }

    std::sort(scores.begin(), scores.end());

    std::vector<int> usedI;
    std::vector<int> usedJ;
    std::vector<TrackedObject> assignedTrackedObjects(trackedObjects.size());
    for (const Score& score : scores)
    {
        if (score.score > 2.0f)
        {
            break;
        }
        int i = score.i;
        int j = score.j;
        bool foundI = (std::find(usedI.begin(), usedI.end(), i) != usedI.end());
        bool foundJ = (std::find(usedJ.begin(), usedJ.end(), j) != usedJ.end());
        if (!foundI && !foundJ)
        {
            assignedTrackedObjects[j] = trackedObjects_[i];
        }
        if (!foundI)
        {
            usedI.push_back(i);
        }
        if (!foundJ)
        {
            usedJ.push_back(j);
        }
    }

    return assignedTrackedObjects;
}

}
