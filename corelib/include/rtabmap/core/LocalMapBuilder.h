#pragma once

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/RayTracing.h>

#include <vector>
#include <utility>
#include <Eigen/Core>

namespace rtabmap {

class LocalMapBuilder
{
public:

	struct Color
	{
		Color()
		{
			missing = true;
		}
		Color(int otherRgb)
		{
			rgb = otherRgb;
			missing = false;
		}
		bool operator==(const Color& other) const
		{
			return (missing == true && other.missing == true) ||
				data == other.data;
		}
		bool operator!=(const Color& other) const
		{
			return !operator==(other);
		}
		int brightness() const
		{
			if (missing)
			{
				return -1;
			}
			return (int)r + (int)g + (int)b;
		}

		union
		{
			union
			{
				struct
				{
					std::uint8_t b;
					std::uint8_t g;
					std::uint8_t r;
					bool missing;
				};
				int rgb;
			};
			int data;
		};

		static const Color missingColor;
	};

	struct ColoredGrid
	{
		int minY;
		int minX;
		cv::Mat grid;
		cv::Mat colors;
	};

	struct LocalMap
	{
		int numEmpty, numObstacles;
		Eigen::Matrix3Xf points;  // z = 0
		std::vector<Color> colors;
	};

public:
	LocalMapBuilder(const ParametersMap& parameters = ParametersMap());
	void parseParameters(const ParametersMap& parameters);

    LocalMap createLocalMap(const Signature& signature) const;

private:
	Eigen::Matrix3Xf convertLaserScan(const LaserScan& laserScan) const;
	Eigen::Matrix3Xf filterMaxRange(const Eigen::Matrix3Xf& points) const;
	Eigen::Matrix3Xf transformPoints(const Eigen::Matrix3Xf& points,
		const Transform& transform) const;
	Eigen::Matrix3Xf getObstaclePoints(const Eigen::Matrix3Xf& points) const;

	std::vector<Color> getPointsColors(const Eigen::Matrix3Xf& points,
		const std::vector<cv::Mat>& images,
		const std::vector<rtabmap::CameraModel>& cameraModels) const;

	ColoredGrid coloredGridFromObstacles(const Eigen::Matrix3Xf& points,
		const std::vector<Color>& colors,
		const Eigen::Vector2f& sensor) const;
	void traceRays(ColoredGrid& coloredGrid,
		const Eigen::Vector2f& sensor) const;
	LocalMap localMapFromColoredGrid(const ColoredGrid& coloredGrid) const;

private:
	float cellSize_;
	float maxRange_;
	float maxRangeSqr_;
	float minObstacleHeight_;
	float maxObstacleHeight_;
	float minSemanticRange_;
	float minSemanticRangeSqr_;
	float maxSemanticRange_;
	float maxSemanticRangeSqr_;
	bool useRayTracing_;

	std::unique_ptr<RayTracing> rayTracing_;
};

}
