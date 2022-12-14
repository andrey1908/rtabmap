#pragma once

#include <rtabmap/core/Parameters.h>

#include <opencv2/opencv.hpp>
#include <vector>
#include <map>

namespace rtabmap {

class SemanticDilation
{
public:
	static const cv::Vec3b backgroundColor;  // (0, 0, 0)

private:
	struct PixelCoords {
		bool inFrame(int h, int w) const {
			return y >= 0 && x >= 0 && y < h && x < w;
		}

		int y;
		int x;
	};

public:
	SemanticDilation(const ParametersMap& parameters = ParametersMap());
	void parseParameters(const ParametersMap& parameters);

	cv::Mat dilate(const cv::Mat& image) const;

	inline int dilationSize() const
	{
		return dilationSize_;
	}

private:
	void computeDilationPixels();

private:
	int dilationSize_;
	int dilationSizeSqr_;
	int dilationWidth_;

	std::vector<PixelCoords> dilationPixels_;
	std::vector<int> dilationWidthToPixelsNum_;
};

}
