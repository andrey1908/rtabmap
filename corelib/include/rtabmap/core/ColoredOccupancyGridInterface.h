#pragma once

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/LocalMapBuilder.h>

#include <list>
#include <map>
#include <utility>
#include <optional>
#include <Eigen/Core>

namespace rtabmap {

class ColoredOccupancyGridInterface
{
public:
	struct OccupancyGrid
	{
		using GridType = Eigen::Matrix<char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
		float minY;
		float minX;
		GridType grid;
	};

	struct ColorGrid
	{
		using GridType = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
		float minY;
		float minX;
		GridType grid;
	};

protected:
	inline static float logodds(float probability)
	{
		return log(probability / (1.0f - probability));
	}

	inline static float probability(float logodds)
	{
		return 1.0f - (1.0f / (1.0f + exp(logodds)));
	}

	struct MapLimits
	{
		MapLimits() :
			minX(std::numeric_limits<int>::max()),
			minY(std::numeric_limits<int>::max()),
			maxX(std::numeric_limits<int>::min() + 1),
			maxY(std::numeric_limits<int>::min() + 1) {}
		bool operator==(const MapLimits& other) const
		{
			return minX == other.minX && minY == other.minY && maxX == other.maxX && maxY == other.maxY;
		}
		bool operator!=(const MapLimits& other) const
		{
			return !(*this == other);
		}
		bool valid() const
		{
			return minX != std::numeric_limits<int>::max();
		}
		void update(int x, int y)
		{
			if (x < minX)
				minX = x;
			if (x > maxX - 1)
				maxX = x + 1;
			if (y < minY)
				minY = y;
			if (y > maxY - 1)
				maxY = y + 1;
		}
		int width() const
		{
			return maxX - minX;
		}
		int height() const
		{
			return maxY - minY;
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
				res.minX = res.maxX;
			if (res.minY > res.maxY)
				res.minY = res.maxY;
			return res;
		}
		int minX;
		int minY;
		int maxX;
		int maxY;
	};

public:
	struct LocalMap : LocalMapBuilder::LocalMap
	{
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
