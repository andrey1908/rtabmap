#pragma once

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SemanticDilation.h>
#include <rtabmap/core/RayTracing.h>

#include <vector>
#include <utility>
#include <Eigen/Core>

namespace rtabmap {

class LocalMapBuilder
{
public:

	class Color
	{
	public:
		static const Color missingColor;

		Color() { missing_ = true; }
		Color(int rgb) { setRgb(rgb); };
		inline bool operator==(const Color& other) const
		{
			return (missing_ && other.missing_) || data_ == other.data_;
		}
		inline bool operator!=(const Color& other) const
		{
			return !operator==(other);
		}
		inline int brightness() const
		{
			if (missing_) return -1;
			return (int)r_ + (int)g_ + (int)b_;
		}
		inline std::uint8_t& b() { return b_; }
		inline std::uint8_t b() const { return b_; }
		inline std::uint8_t& g() { return g_; }
		inline std::uint8_t g() const { return g_; }
		inline std::uint8_t& r() { return r_; }
		inline std::uint8_t r() const { return r_; }
		inline bool& missing() { return missing_; }
		inline bool missing() const { return missing_; }
		inline int rgb() const { return rgb_; }
		inline void setRgb(int rgb) { rgb_ = rgb; missing_ = false; };
		inline int data() const { return data_; }

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

	std::unique_ptr<SemanticDilation> semanticDilation_;
	std::unique_ptr<RayTracing> rayTracing_;
};

}
