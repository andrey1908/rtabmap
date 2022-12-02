#include <rtabmap/core/DoorTracking.h>
#include <rtabmap/utilite/ULogger.h>
#include "time_measurer/time_measurer.h"

DoorTracking::DoorTracking(int smallRadius, int largeRadius) :
        smallRadius_(smallRadius), largeRadius_(largeRadius) {
    smallRadiusSqr_ = smallRadius * smallRadius;
    doubleSmallRadius_ = smallRadius * 2;
    doubleSmallRadiusSqr_ = doubleSmallRadius_ * doubleSmallRadius_;
    largeRadiusSqr_ = largeRadius * largeRadius;
    doubleLargeRadius_ = largeRadius * 2;
    doubleLargeRadiusSqr_ = doubleLargeRadius_ * doubleLargeRadius_;
    precomputeCellToCheckForOccupation();
}

inline int DoorTracking::cellsDistanceSqr(const Cell& a, const Cell& b) {
    int yDistance = a.first - b.first;
    int xDistance = a.second - b.second;
    int distanceSqr = yDistance * yDistance + xDistance * xDistance;
    return distanceSqr;
}

void DoorTracking::precomputeCellToCheckForOccupation() {
    for (int y = -doubleLargeRadius_; y <= doubleLargeRadius_; y++) {
        for (int x = -doubleLargeRadius_; x <= doubleLargeRadius_; x++) {
            if (y * y + x * x <= doubleLargeRadiusSqr_) {
                cellToCheckForOccupation_.emplace_back(y, x);
            }
        }
    }
}

std::vector<DoorTracking::Cell>
DoorTracking::getOccupiedCells(const cv::Mat& image, const Cell& estimation) {
    MEASURE_BLOCK_TIME(getOccupiedCells);
    std::vector<Cell> occupiedCells;
    int ey = estimation.first;
    int ex = estimation.second;
    for (const auto& cell : cellToCheckForOccupation_) {
        int cy = cell.first + ey;
        int cx = cell.second + ex;
        if (cy < 0 || cx < 0 || cy >= image.rows || cx >= image.cols) {
            continue;
        }
        if (image.at<std::uint8_t>(cy, cx) == occupiedCellValue) {
            occupiedCells.emplace_back(cy, cx);
        }
    }
    return occupiedCells;
}

std::vector<DoorTracking::Segment> DoorTracking::segmentation(const std::vector<Cell>& cells) {
    MEASURE_BLOCK_TIME(segmentation);
    std::vector<Segment> segments;
    std::vector<int> notUsedCellIndices;
    notUsedCellIndices.reserve(cells.size());
    for (int i = 0; i < cells.size(); i++) {
        notUsedCellIndices.push_back(i);
    }
    while (notUsedCellIndices.size()) {
        Segment& segment = segments.emplace_back();
        segment.reserve(cells.size());
        std::deque<int> nextCellIndices;
        int newCellI = notUsedCellIndices.back();
        notUsedCellIndices.pop_back();
        nextCellIndices.push_back(newCellI);
        segment.push_back(cells[newCellI]);
        while (nextCellIndices.size()) {
            int nextCellI = nextCellIndices.front();
            nextCellIndices.pop_front();
            std::vector<int> newCellIndices;
            const Cell& nextCell = cells[nextCellI];
            for (auto it = notUsedCellIndices.end() - 1; it >= notUsedCellIndices.begin();) {
                int notUsedCellI = *it;
                const Cell& notUsedCell = cells[notUsedCellI];
                int distanceSqr = cellsDistanceSqr(nextCell, notUsedCell);
                if (distanceSqr <= doubleSmallRadiusSqr_) {
                    auto removeIt = it;
                    --it;
                    notUsedCellIndices.erase(removeIt);
                    nextCellIndices.push_back(notUsedCellI);
                    segment.push_back(notUsedCell);
                } else {
                    --it;
                }
            }
        }
    }
    return segments;
}

std::pair<DoorTracking::Cell, DoorTracking::Cell>
DoorTracking::findClosestCellsInSegments(const Segment& segment1, const Segment& segment2) {
    MEASURE_BLOCK_TIME(findClosestCellsInSegments);
    int minDistanceSqr = std::numeric_limits<int>::max();
    Cell closestCell1;
    Cell closestCell2;
    for (int i = 0; i < segment1.size(); i++) {
        for (int j = 0; j < segment2.size(); j++) {
            const Cell& cell1 = segment1[i];
            const Cell& cell2 = segment2[j];
            int distanceSqr = cellsDistanceSqr(cell1, cell2);
            if (distanceSqr < minDistanceSqr) {
                closestCell1 = cell1;
                closestCell2 = cell2;
                minDistanceSqr = distanceSqr;
            }
        }
    }
    return std::make_pair(closestCell1, closestCell2);
}

std::pair<DoorTracking::Cell, DoorTracking::Cell>
DoorTracking::trackDoor(const cv::Mat& image, const Cell& estimation) {
    MEASURE_BLOCK_TIME(trackDoor);
    const std::vector<Cell>& occupiedCells = getOccupiedCells(image, estimation);
    const std::vector<Segment>& segments = segmentation(occupiedCells);
    if (segments.size() != 2) {
        UWARN("Found %d segments", segments.size());
        return std::make_pair(DoorTracking::Cell(-1, -1), DoorTracking::Cell(-1, -1));
    } else {
        UINFO("Detected!");
    }
    const std::pair<Cell, Cell>& doorCorners = findClosestCellsInSegments(segments[0], segments[1]);
    return doorCorners;
}
