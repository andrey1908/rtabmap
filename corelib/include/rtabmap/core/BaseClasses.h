#pragma once

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Transform.h>

#include <list>
#include <map>
#include <utility>
#include <optional>
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

	struct MapLimits
	{
		MapLimits() :
			minX(std::numeric_limits<int>::max()),
			minY(std::numeric_limits<int>::max()),
			maxX(std::numeric_limits<int>::min()),
			maxY(std::numeric_limits<int>::min()) {}
		bool operator==(const MapLimits& other) const
		{
			return
				minX == other.minX &&
				minY == other.minY &&
				maxX == other.maxX &&
				maxY == other.maxY;
		}
		bool operator!=(const MapLimits& other) const
		{
			return !operator==(other);
		}
		bool valid() const
		{
			return minX != std::numeric_limits<int>::max();
		}
		void update(int x, int y)
		{
			if (x < minX)
				minX = x;
			if (x > maxX)
				maxX = x;
			if (y < minY)
				minY = y;
			if (y > maxY)
				maxY = y;
		}
		int width() const
		{
			return maxX - minX + 1;
		}
		int height() const
		{
			return maxY - minY + 1;
		}
		static MapLimits unite(const MapLimits& a, const MapLimits& b)
		{
			MapLimits res;
			res.minX = std::min(a.minX, b.minX);
			res.minY = std::min(a.minY, b.minY);
			res.maxX = std::max(a.maxX, b.maxX);
			res.maxY = std::max(a.maxY, b.maxY);
			return res;
		}
		static MapLimits intersect(const MapLimits& a, const MapLimits& b)
		{
			MapLimits res;
			res.minX = std::max(a.minX, b.minX);
			res.minY = std::max(a.minY, b.minY);
			res.maxX = std::min(a.maxX, b.maxX);
			res.maxY = std::min(a.maxY, b.maxY);
			if (res.minX > res.maxX)
				res.minX = res.maxX + 1;
			if (res.minY > res.maxY)
				res.minY = res.maxY + 1;
			return res;
		}
		int minX;
		int minY;
		int maxX;
		int maxY;
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
		int numEmpty, numObstacles;
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
		Node(LocalMapType&& otherLocalMap,
			TransformedLocalMapType&& otherTransformedLocalMap) :
				localMap(std::forward<LocalMapType>(otherLocalMap)),
				transformedLocalMap(
					std::forward<TransformedLocalMapType>(otherTransformedLocalMap)) {}
		const LocalMap localMap;
		std::optional<TransformedLocalMap> transformedLocalMap;
	};
};

}
