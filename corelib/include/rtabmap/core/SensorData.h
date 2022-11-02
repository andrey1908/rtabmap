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

#ifndef SENSORDATA_H_
#define SENSORDATA_H_

#include <rtabmap/core/RtabmapExp.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/StereoCameraModel.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/IMU.h>
#include <rtabmap/core/GPS.h>
#include <rtabmap/core/EnvSensor.h>
#include <rtabmap/core/Landmark.h>
#include <rtabmap/core/GlobalDescriptor.h>

namespace rtabmap
{

/**
 * An id is automatically generated if id=0.
 */
class RTABMAP_EXP SensorData
{
public:
	// empty constructor
	SensorData();

	// Appearance-only constructor
	SensorData(
			const cv::Mat & image,
			int id = 0,
			double stamp = 0.0);

	// Mono constructor
	SensorData(
			const cv::Mat & image,
			const CameraModel & cameraModel,
			int id = 0,
			double stamp = 0.0);

	// RGB-D constructor
	SensorData(
			const cv::Mat & rgb,
			const cv::Mat & depth,
			const CameraModel & cameraModel,
			int id = 0,
			double stamp = 0.0);

	// RGB-D constructor + laser scan
	SensorData(
			const LaserScan & laserScan,
			const cv::Mat & rgb,
			const cv::Mat & depth,
			const CameraModel & cameraModel,
			int id = 0,
			double stamp = 0.0);

	// Multi-cameras RGB-D constructor
	SensorData(
			const std::vector<cv::Mat> & rgbs,
			const std::vector<cv::Mat> & depths,
			const std::vector<CameraModel> & cameraModels,
			int id = 0,
			double stamp = 0.0);

	// Multi-cameras RGB-D constructor + laser scan
	SensorData(
			const LaserScan & laserScan,
			const std::vector<cv::Mat> & rgbs,
			const std::vector<cv::Mat> & depths,
			const std::vector<CameraModel> & cameraModels,
			int id = 0,
			double stamp = 0.0);

	// Stereo constructor
	SensorData(
			const cv::Mat & left,
			const cv::Mat & right,
			const StereoCameraModel & cameraModel,
			int id = 0,
			double stamp = 0.0);

	// Stereo constructor + laser scan
	SensorData(
			const LaserScan & laserScan,
			const cv::Mat & left,
			const cv::Mat & right,
			const StereoCameraModel & cameraModel,
			int id = 0,
			double stamp = 0.0);

	// IMU constructor
	SensorData(
			const IMU & imu,
			int id = 0,
			double stamp = 0.0);

	virtual ~SensorData();

	bool isValid() const {
		return !(_id == 0 &&
			_sec == 0 &&
			_nsec == 0 &&
			_images.empty() &&
			_depthsOrRights.empty() &&
			_laserScan.isEmpty() &&
			_cameraModels.size() == 0 &&
			!_stereoCameraModel.isValidForProjection() &&
			_keypoints.size() == 0 &&
			_descriptors.empty() &&
			imu_.empty());
	}

	int id() const {return _id;}
	void setId(int id) {_id = id;}
	double stamp() const {return uSecNSecStamp2Double(_sec, _nsec);}
	uint32_t sec() const {return _sec;}
	uint32_t nsec() const {return _nsec;}
	void setStamp(double stamp) {std::tie(_sec, _nsec) = uDoubleStamp2SecNSec(stamp);}
	void setStamp(uint32_t sec, uint32_t nsec) {_sec = sec; _nsec = nsec;}

	cv::Mat image() const {return _images.size()?_images[0]:cv::Mat();}
	const std::vector<cv::Mat> & images() const {return _images;}
	cv::Mat depthOrRight() const {return _depthsOrRights.size()?_depthsOrRights[0]:cv::Mat();}
	const std::vector<cv::Mat> & depthsOrRights() const {return _depthsOrRights;}
	const LaserScan & laserScan() const {return _laserScan;}

	/**
	 * Set image data.
	 */
	void setRGBImage(const cv::Mat & rgb, const CameraModel & model);
	void setRGBImages(const std::vector<cv::Mat> & rgbs, const std::vector<CameraModel> & models);
	void setRGBDImage(const cv::Mat & rgb, const cv::Mat & depth, const CameraModel & model);
	void setRGBDImage(const cv::Mat & rgb, const cv::Mat & depth, const CameraModel & model, const CameraModel & depthModel);
	void setRGBDImages(const std::vector<cv::Mat> & rgbs, const std::vector<cv::Mat> & depths,
		const std::vector<CameraModel> & models);
	void setRGBDImages(const std::vector<cv::Mat> & rgbs, const std::vector<cv::Mat> & depths,
		const std::vector<CameraModel> & models, const std::vector<CameraModel> & depthModels);
	void setStereoImage(const cv::Mat & left, const cv::Mat & right, const StereoCameraModel & stereoCameraModel);

	/**
	 * Set laser scan data.
	 */
	void setLaserScan(const LaserScan & laserScan);

	void setCameraModel(const CameraModel & model) {_cameraModels.clear(); _cameraModels.push_back(model);}
	void setCameraModels(const std::vector<CameraModel> & models) {_cameraModels = models;}
	void setStereoCameraModel(const StereoCameraModel & stereoCameraModel) {_stereoCameraModel = stereoCameraModel;}

	// for convenience
	cv::Mat depth() const {return (_depthsOrRights.size() && _depthsOrRights[0].type()!=CV_8UC1)?_depthsOrRights[0]:cv::Mat();}
	cv::Mat right() const {return (_depthsOrRights.size() && _depthsOrRights[0].type()==CV_8UC1)?_depthsOrRights[0]:cv::Mat();}

	RTABMAP_DEPRECATED(void setImage(const cv::Mat & image), "Use setRGBDImage() or setStereoImage().");
	RTABMAP_DEPRECATED(void setDepthOrRight(const cv::Mat & image), "Use setRGBDImage() or setStereoImage().");

	const std::vector<CameraModel> & cameraModels() const {return _cameraModels;}
	const StereoCameraModel & stereoCameraModel() const {return _stereoCameraModel;}

	void setFeatures(const std::vector<cv::KeyPoint> & keypoints, const std::vector<cv::Point3f> & keypoints3D, const cv::Mat & descriptors);
	const std::vector<cv::KeyPoint> & keypoints() const {return _keypoints;}
	const std::vector<cv::Point3f> & keypoints3D() const {return _keypoints3D;}
	const cv::Mat & descriptors() const {return _descriptors;}

	void addGlobalDescriptor(const GlobalDescriptor & descriptor) {_globalDescriptors.push_back(descriptor);}
	void setGlobalDescriptors(const std::vector<GlobalDescriptor> & descriptors) {_globalDescriptors = descriptors;}
	void clearGlobalDescriptors() {_globalDescriptors.clear();}
	const std::vector<GlobalDescriptor> & globalDescriptors() const {return _globalDescriptors;}

	void setGroundTruth(const Transform & pose) {groundTruth_ = pose;}
	const Transform & groundTruth() const {return groundTruth_;}

	void setGlobalPose(const Transform & pose, const cv::Mat & covariance) {globalPose_ = pose; globalPoseCovariance_ = covariance;}
	const Transform & globalPose() const {return globalPose_;}
	const cv::Mat & globalPoseCovariance() const {return globalPoseCovariance_;}

	void setGPS(const GPS & gps) {gps_ = gps;}
	const GPS & gps() const {return gps_;}

	void setIMU(const IMU & imu) {imu_ = imu; }
	const IMU & imu() const {return imu_;}

	void setEnvSensors(const EnvSensors & sensors) {_envSensors = sensors;}
	void addEnvSensor(const EnvSensor & sensor) {_envSensors.insert(std::make_pair(sensor.type(), sensor));}
	const EnvSensors & envSensors() const {return _envSensors;}

	void setLandmarks(const Landmarks & landmarks) {_landmarks = landmarks;}
	const Landmarks & landmarks() const {return _landmarks;}

	void clearData(bool images = true, bool scan = true);

	bool isPointVisibleFromCameras(const cv::Point3f & pt) const; // assuming point is in robot frame

private:
	int _id;
	uint32_t _sec;
	uint32_t _nsec;

	std::vector<cv::Mat> _images;          // CV_8UC1 or CV_8UC3
	std::vector<cv::Mat> _depthsOrRights;   // depth CV_16UC1 or CV_32FC1, right image CV_8UC1
	LaserScan _laserScan;

	std::vector<CameraModel> _cameraModels;
	std::vector<CameraModel> _depthCameraModels;
	StereoCameraModel _stereoCameraModel;

	// environmental sensors
	EnvSensors _envSensors;

	// landmarks
	Landmarks _landmarks;

	// features
	std::vector<cv::KeyPoint> _keypoints;
	std::vector<cv::Point3f> _keypoints3D;
	cv::Mat _descriptors;

	// global descriptors
	std::vector<GlobalDescriptor> _globalDescriptors;

	Transform groundTruth_;

	Transform globalPose_;
	cv::Mat globalPoseCovariance_; // 6x6 double

	GPS gps_;

	IMU imu_;
};

}


#endif /* SENSORDATA_H_ */
