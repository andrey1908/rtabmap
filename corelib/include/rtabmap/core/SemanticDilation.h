#pragma once

#include <rtabmap/core/Parameters.h>

#include <opencv2/opencv.hpp>
#include <vector>
#include <map>

namespace rtabmap {

class SemanticDilation
{
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
	SemanticDilation(int dilationSize);
	void parseParameters(const ParametersMap& parameters);
	void parseParameters(int dilationSize);

	cv::Mat dilate(const cv::Mat& image,
		const cv::Vec3b& backgroundColor = cv::Vec3b(0, 0, 0)) const;

	inline int dilationSize() const
	{
		return dilationSize_;
	}

private:
	void initialize();
	void computeDilationPixels();

private:
	int dilationSize_;
	int dilationSizeSqr_;
	int dilationWidth_;

	std::vector<PixelCoords> dilationPixels_;
	std::vector<int> dilationWidthToPixelsNum_;
};

}
