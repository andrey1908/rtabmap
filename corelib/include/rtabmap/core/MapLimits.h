#pragma once

#include <limits>
#include <algorithm>

namespace rtabmap {

class MapLimits
{
public:
	MapLimits() :
		minX_(std::numeric_limits<int>::max()),
		minY_(std::numeric_limits<int>::max()),
		maxX_(std::numeric_limits<int>::min()),
		maxY_(std::numeric_limits<int>::min()) {}
	int minX() const {
		return minX_;
	}
	int minY() const {
		return minY_;
	}
	int maxX() const {
		return maxX_;
	}
	int maxY() const {
		return maxY_;
	}
	bool operator==(const MapLimits& other) const
	{
		return
			minX_ == other.minX_ &&
			minY_ == other.minY_ &&
			maxX_ == other.maxX_ &&
			maxY_ == other.maxY_;
	}
	bool operator!=(const MapLimits& other) const
	{
		return !operator==(other);
	}
	bool valid() const
	{
		return minX_ != std::numeric_limits<int>::max();
	}
	void update(int x, int y)
	{
		if (x < minX_)
			minX_ = x;
		if (x > maxX_)
			maxX_ = x;
		if (y < minY_)
			minY_ = y;
		if (y > maxY_)
			maxY_ = y;
	}
	int width() const
	{
		return maxX_ - minX_ + 1;
	}
	int height() const
	{
		return maxY_ - minY_ + 1;
	}
	static MapLimits unite(const MapLimits& a, const MapLimits& b)
	{
		MapLimits res;
		res.minX_ = std::min(a.minX_, b.minX_);
		res.minY_ = std::min(a.minY_, b.minY_);
		res.maxX_ = std::max(a.maxX_, b.maxX_);
		res.maxY_ = std::max(a.maxY_, b.maxY_);
		return res;
	}
	static MapLimits intersect(const MapLimits& a, const MapLimits& b)
	{
		MapLimits res;
		res.minX_ = std::max(a.minX_, b.minX_);
		res.minY_ = std::max(a.minY_, b.minY_);
		res.maxX_ = std::min(a.maxX_, b.maxX_);
		res.maxY_ = std::min(a.maxY_, b.maxY_);
		if (res.minX_ > res.maxX_)
			res.minX_ = res.maxX_ + 1;
		if (res.minY_ > res.maxY_)
			res.minY_ = res.maxY_ + 1;
		return res;
	}

private:
	int minX_;
	int minY_;
	int maxX_;
	int maxY_;
};

}