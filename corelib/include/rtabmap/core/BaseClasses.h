#pragma once

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Transform.h>

#include <list>
#include <map>
#include <utility>
#include <optional>
#include <memory>
#include <Eigen/Core>

namespace rtabmap {

class BaseClasses
{
public:
	class Color
	{
	public:
		static const Color missingColor;

		Color() { missing_ = true; }
		Color(int rgb) { setRgb(rgb); };
		bool operator==(const Color& other) const
		{
			return (missing_ && other.missing_) || data_ == other.data_;
		}
		bool operator!=(const Color& other) const
		{
			return !operator==(other);
		}
		int brightness() const
		{
			if (missing_) return -1;
			return (int)r_ + (int)g_ + (int)b_;
		}
		std::uint8_t& b() { return b_; }
		std::uint8_t b() const { return b_; }
		std::uint8_t& g() { return g_; }
		std::uint8_t g() const { return g_; }
		std::uint8_t& r() { return r_; }
		std::uint8_t r() const { return r_; }
		bool& missing() { return missing_; }
		bool missing() const { return missing_; }
		int rgb() const { return rgb_; }
		void setRgb(int rgb) { rgb_ = rgb; missing_ = false; };
		int data() const { return data_; }

	private:
		union
		{
			union
			{
				struct
				{
					std::uint8_t b_;
					std::uint8_t g_;
					std::uint8_t r_;
					bool missing_;
				};
				int rgb_;
			};
			int data_;
		};
	};

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

	struct OccupancyGrid
	{
		using GridType = Eigen::Matrix<char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
		MapLimits limits;
		GridType grid;
	};

	struct ColorGrid
	{
		using GridType = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
		MapLimits limits;
		GridType grid;
	};

	inline static float logodds(float probability)
	{
		return log(probability / (1.0f - probability));
	}

	inline static float probability(float logodds)
	{
		return 1.0f - (1.0f / (1.0f + exp(logodds)));
	}

	struct LocalMap
	{
		int numObstacles;
		int numEmpty;
		Eigen::Matrix3Xf points;  // z = 0
		std::vector<Color> colors;

		float sensorBlindRange2dSqr;
		Transform toSensor;
	};

	struct TransformedLocalMap
	{
		Transform pose;
		Eigen::Matrix2Xi points;
		MapLimits mapLimits;
	};

	struct Node
	{
		template <typename LocalMapType, typename TransformedLocalMapType>
		Node(
			const std::shared_ptr<LocalMapType>& otherLocalMap,
			TransformedLocalMapType&& otherTransformedLocalMap) :
				localMap(otherLocalMap),
				transformedLocalMap(
					std::forward<TransformedLocalMapType>(otherTransformedLocalMap)) {}
		const std::shared_ptr<const LocalMap> localMap;
		std::optional<TransformedLocalMap> transformedLocalMap;
	};
};

}
