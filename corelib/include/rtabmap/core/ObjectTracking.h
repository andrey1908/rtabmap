#pragma once

#include <rtabmap/core/MapLimits.h>
#include <rtabmap/core/LocalMap.h>

#include <vector>
#include <utility>
#include <functional>

namespace rtabmap {

class ObjectTracking
{
public:
    struct Cell
    {
        Cell(int otherY, int otherX) :
            y(otherY),
            x(otherX) {}

        static int distanceSqr(const Cell& a, const Cell& b)
        {
            int dy = a.y - b.y;
            int dx = a.x - b.x;
            return dy * dy + dx * dx;
        }

        Cell operator+(const Cell& other)
        {
            return Cell(y + other.y, x + other.x);
        }

        int y;
        int x;
    };

    struct Point
    {
        Point() : x(0.0f), y(0.0f) {}
        Point(float otherX, float otherY) :
            x(otherX),
            y(otherY) {}

        static float distanceSqr(const Point& a, const Point& b)
        {
            float dx = a.x - b.x;
            float dy = a.y - b.y;
            return dx * dx + dy * dy;
        }

        float x;
        float y;
    };

    struct Velocity
    {
        Velocity() : vx(0.0f), vy(0.0f) {}

        float vx;
        float vy;
    };

    struct TrackedObject
    {
        std::int32_t id = -1;
        Point position;
        Velocity velocity;
        std::vector<Cell> object;
    };

private:
    // row-major order
    class Indexing
    {
    public:
        Indexing(int rows, int cols) :
            rows_(rows),
            cols_(cols) {}
        
        int index(int i, int j) const { return i * cols_ + j; }
        int row(int index) const { return index / cols_; }
        int col(int index) const { return index % cols_; }
        int total() const { return rows_ * cols_; }
    
    private:
        int rows_;
        int cols_;
    };

    struct Score
    {
        bool operator<(const Score& other) { return score < other.score; }

        float score;
        int i;
        int j;
    };

public:
    ObjectTracking(float cellSize) : cellSize_(cellSize) {};

    void track(const LocalMap& localMap, const Transform& pose);

    const std::vector<TrackedObject>& trackedObjects() { return trackedObjects_; }

public:
    std::vector<TrackedObject> detect(
        const LocalMap& localMap, const Transform& pose);
    TrackedObject segment(cv::Mat& colorGrid, const Cell& startCell,
        const MapLimitsI& mapLimits, const Transform& pose);
    std::vector<TrackedObject> assign(const std::vector<TrackedObject>& trackedObjects);

private:
    float cellSize_;
    std::vector<TrackedObject> trackedObjects_;

    std::int32_t nextTrackedId_ = 0;
};

}
