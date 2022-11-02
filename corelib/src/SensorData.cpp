/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/utilite/ULogger.h"
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UConversion.h>

namespace rtabmap
{

// empty constructor
SensorData::SensorData() :
		_id(0),
		_sec(0),
		_nsec(0)
{
}

// Appearance-only constructor
SensorData::SensorData(
		const cv::Mat & image,
		int id,
		double stamp,
		const cv::Mat & userData) :
		_id(id),
		_sec(0),
		_nsec(0)
{
	std::tie(_sec, _nsec) = uDoubleStamp2SecNSec(stamp);
	setRGBDImage(image, cv::Mat(), CameraModel());
}

// Mono constructor
SensorData::SensorData(
		const cv::Mat & image,
		const CameraModel & cameraModel,
		int id,
		double stamp,
		const cv::Mat & userData) :
		_id(id),
		_sec(0),
		_nsec(0)
{
	std::tie(_sec, _nsec) = uDoubleStamp2SecNSec(stamp);
	setRGBDImage(image, cv::Mat(), cameraModel);
}

// RGB-D constructor
SensorData::SensorData(
		const cv::Mat & rgb,
		const cv::Mat & depth,
		const CameraModel & cameraModel,
		int id,
		double stamp,
		const cv::Mat & userData) :
		_id(id),
		_sec(0),
		_nsec(0)
{
	std::tie(_sec, _nsec) = uDoubleStamp2SecNSec(stamp);
	setRGBDImage(rgb, depth, cameraModel);
}

// RGB-D constructor + laser scan
SensorData::SensorData(
		const LaserScan & laserScan,
		const cv::Mat & rgb,
		const cv::Mat & depth,
		const CameraModel & cameraModel,
		int id,
		double stamp,
		const cv::Mat & userData) :
		_id(id),
		_sec(0),
		_nsec(0)
{
	std::tie(_sec, _nsec) = uDoubleStamp2SecNSec(stamp);
	setRGBDImage(rgb, depth, cameraModel);
	setLaserScan(laserScan);
}

// Multi-cameras RGB-D constructor
SensorData::SensorData(
		const cv::Mat & rgb,
		const cv::Mat & depth,
		const std::vector<CameraModel> & cameraModels,
		int id,
		double stamp,
		const cv::Mat & userData) :
		_id(id),
		_sec(0),
		_nsec(0)
{
	std::tie(_sec, _nsec) = uDoubleStamp2SecNSec(stamp);
	setRGBDImage(rgb, depth, cameraModels);
}

// Multi-cameras RGB-D constructor + laser scan
SensorData::SensorData(
		const LaserScan & laserScan,
		const cv::Mat & rgb,
		const cv::Mat & depth,
		const std::vector<CameraModel> & cameraModels,
		int id,
		double stamp,
		const cv::Mat & userData) :
		_id(id),
		_sec(0),
		_nsec(0)
{
	std::tie(_sec, _nsec) = uDoubleStamp2SecNSec(stamp);
	setRGBDImage(rgb, depth, cameraModels);
	setLaserScan(laserScan);
}

// Stereo constructor
SensorData::SensorData(
		const cv::Mat & left,
		const cv::Mat & right,
		const StereoCameraModel & cameraModel,
		int id,
		double stamp,
		const cv::Mat & userData):
		_id(id),
		_sec(0),
		_nsec(0)
{
	std::tie(_sec, _nsec) = uDoubleStamp2SecNSec(stamp);
	setStereoImage(left, right, cameraModel);
}

// Stereo constructor + 2d laser scan
SensorData::SensorData(
		const LaserScan & laserScan,
		const cv::Mat & left,
		const cv::Mat & right,
		const StereoCameraModel & cameraModel,
		int id,
		double stamp,
		const cv::Mat & userData) :
		_id(id),
		_sec(0),
		_nsec(0)
{
	std::tie(_sec, _nsec) = uDoubleStamp2SecNSec(stamp);
	setStereoImage(left, right, cameraModel);
	setLaserScan(laserScan);
}

SensorData::SensorData(
	const IMU & imu,
	int id,
	double stamp) :
		_id(id),
		_sec(0),
		_nsec(0)
{
	std::tie(_sec, _nsec) = uDoubleStamp2SecNSec(stamp);
	imu_ = imu;
}

SensorData::~SensorData()
{
}

void SensorData::setRGBDImage(
		const cv::Mat & rgb,
		const cv::Mat & depth,
		const CameraModel & model)
{
	std::vector<CameraModel> models;
	models.push_back(model);
	setRGBDImage(rgb, depth, models);
}
void SensorData::setRGBDImage(
		const cv::Mat & rgb,
		const cv::Mat & depth,
		const std::vector<CameraModel> & models)
{
	_stereoCameraModel = StereoCameraModel();
	_cameraModels = models;
	if(!rgb.empty())
	{
		UASSERT(rgb.type() == CV_8UC1 || // Mono
				rgb.type() == CV_8UC3);  // RGB
		_image = rgb;
	}
	else
	{
		_image = cv::Mat();
	}

	if(!depth.empty())
	{
		UASSERT(depth.type() == CV_32FC1 || // Depth in meter
				depth.type() == CV_16UC1); // Depth in millimetre
		_depthOrRight = depth;
	}
	else
	{
		_depthOrRight = cv::Mat();
	}
}
void SensorData::setStereoImage(
		const cv::Mat & left,
		const cv::Mat & right,
		const StereoCameraModel & stereoCameraModel)
{
	_cameraModels.clear();
	_stereoCameraModel = stereoCameraModel;

	if(!left.empty())
	{
		UASSERT(left.type() == CV_8UC1 || // Mono
				left.type() == CV_8UC3);  // RGB
		_image = left;
	}
	else
	{
		_image = cv::Mat();
	}

	if(!right.empty())
	{
		UASSERT(right.type() == CV_8UC1); // Mono
		_depthOrRight = right;
	}
	else
	{
		_depthOrRight = cv::Mat();
	}
}
void SensorData::setLaserScan(const LaserScan & laserScan)
{
	_laserScan = laserScan;
}

void SensorData::setImage(const cv::Mat & image)
{
	UASSERT(image.empty() || image.rows > 1);
	_image = image;
}
void SensorData::setDepthOrRight(const cv::Mat & image)
{
	UASSERT(image.empty() || image.rows > 1);
	_depthOrRight = image;
}

void SensorData::setFeatures(const std::vector<cv::KeyPoint> & keypoints, const std::vector<cv::Point3f> & keypoints3D, const cv::Mat & descriptors)
{
	UASSERT_MSG(keypoints3D.empty() || keypoints.size() == keypoints3D.size(), uFormat("keypoints=%d keypoints3D=%d", (int)keypoints.size(), (int)keypoints3D.size()).c_str());
	UASSERT_MSG(descriptors.empty() || (int)keypoints.size() == descriptors.rows, uFormat("keypoints=%d descriptors=%d", (int)keypoints.size(), descriptors.rows).c_str());
	_keypoints = keypoints;
	_keypoints3D = keypoints3D;
	_descriptors = descriptors;
}

void SensorData::clearData(bool images, bool scan)
{
	if(images)
	{
		_image = cv::Mat();
		_depthOrRight = cv::Mat();
	}
	if(scan)
	{
		_laserScan.clear();
	}
}


bool SensorData::isPointVisibleFromCameras(const cv::Point3f & pt) const
{
	if(_cameraModels.size() >= 1)
	{
		for(unsigned int i=0; i<_cameraModels.size(); ++i)
		{
			if(_cameraModels[i].isValidForProjection() && !_cameraModels[i].localTransform().isNull())
			{
				cv::Point3f ptInCameraFrame = util3d::transformPoint(pt, _cameraModels[i].localTransform().inverse());
				if(ptInCameraFrame.z > 0.0f)
				{
					int borderWidth = int(float(_cameraModels[i].imageWidth())* 0.2);
					int u, v;
					_cameraModels[i].reproject(ptInCameraFrame.x, ptInCameraFrame.y, ptInCameraFrame.z, u, v);
					if(uIsInBounds(u, borderWidth, _cameraModels[i].imageWidth()-2*borderWidth) &&
					   uIsInBounds(v, borderWidth, _cameraModels[i].imageHeight()-2*borderWidth))
					{
						return true;
					}
				}
			}
		}
	}
	else if(_stereoCameraModel.isValidForProjection())
	{
		cv::Point3f ptInCameraFrame = util3d::transformPoint(pt, _stereoCameraModel.localTransform().inverse());
		if(ptInCameraFrame.z > 0.0f)
		{
			int u, v;
			_stereoCameraModel.left().reproject(ptInCameraFrame.x, ptInCameraFrame.y, ptInCameraFrame.z, u, v);
			return uIsInBounds(u, 0, _stereoCameraModel.left().imageWidth()) &&
				   uIsInBounds(v, 0, _stereoCameraModel.left().imageHeight());
		}
	}
	else
	{
		UERROR("no valid camera model!");
	}
	return false;
}

} // namespace rtabmap

