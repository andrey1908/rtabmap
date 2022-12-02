#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <utility>

class DoorTracking {
private:
    static constexpr std::uint8_t occupiedCellValue = 100;

public:
    using Cell = std::pair<int, int>;  // (y, x)
    using Segment = std::vector<Cell>;

    DoorTracking(int smallRadius, int largeRadius);
    ~DoorTracking() {};

    std::pair<Cell, Cell> trackDoor(const cv::Mat& image, const Cell& estimation);

private:
    inline int cellsDistanceSqr(const Cell& a, const Cell& b);

    void precomputeCellToCheckForOccupation();

    std::vector<Cell> getOccupiedCells(const cv::Mat& image, const Cell& estimation);
    std::vector<Segment> segmentation(const std::vector<Cell>& cells);
    std::pair<Cell, Cell> findClosestCellsInSegments(const Segment& segment1, const Segment& segment2);

private:
    int smallRadius_;
    int smallRadiusSqr_;
    int doubleSmallRadius_;
    int doubleSmallRadiusSqr_;
    int largeRadius_;
    int largeRadiusSqr_;
    int doubleLargeRadius_;
    int doubleLargeRadiusSqr_;

    std::vector<Cell> cellToCheckForOccupation_;
};
