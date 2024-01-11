#include <rtabmap/core/SensorData.h>

#include <cstring>

namespace rtabmap {

proto::CameraParameters toProto(const SensorData::CameraParameters& parameters)
{
    proto::CameraParameters proto;
    proto.set_width(parameters.width);
    proto.set_height(parameters.height);
    proto.set_fx(parameters.fx);
    proto.set_fy(parameters.fy);
    proto.set_cx(parameters.cx);
    proto.set_cy(parameters.cy);
    return proto;
}

SensorData::CameraParameters fromProto(const proto::CameraParameters& proto)
{
    SensorData::CameraParameters parameters;
    parameters.width = proto.width();
    parameters.height = proto.height();
    parameters.fx = proto.fx();
    parameters.fy = proto.fy();
    parameters.cx = proto.cx();
    parameters.cy = proto.cy();
    return parameters;
}

proto::CameraImage toProto(const cv::Mat& image)
{
    proto::CameraImage proto;
    proto.set_rows(image.rows);
    proto.set_cols(image.cols);
    proto.set_type(image.type());
    proto.set_data(image.data, image.elemSize() * image.total());
    return proto;
}

cv::Mat fromProto(const proto::CameraImage& proto)
{
    int rows = proto.rows();
    int cols = proto.cols();
    int type = proto.type();
    cv::Mat image(rows, cols, type);

    UASSERT(proto.data().size() == image.elemSize() * image.total());
    std::memcpy(image.data, (const void*)proto.data().c_str(), proto.data().size());

    return image;
}

proto::CameraData toProto(const SensorData::CameraData& cameraData)
{
    proto::CameraData proto;
    *proto.mutable_to_sensor() = toProto(cameraData.toSensor);
    *proto.mutable_parameters() = toProto(cameraData.parameters);
    *proto.mutable_image() = toProto(cameraData.image);
    return proto;
}

SensorData::CameraData fromProto(const proto::CameraData& proto)
{
    Transform toSensor = fromProto(proto.to_sensor());
    SensorData::CameraParameters parameters = fromProto(proto.parameters());
    cv::Mat image = fromProto(proto.image());

    SensorData::CameraData cameraData{std::move(toSensor), parameters, image};
    return cameraData;
}

proto::LidarData toProto(const SensorData::LidarData& lidarData)
{
    proto::LidarData proto;
    *proto.mutable_to_sensor() = toProto(lidarData.toSensor);

    const Eigen::Matrix3Xf& points = lidarData.points;
    for (int i = 0; i < points.cols(); i++)
    {
        float x = points(0, i);
        float y = points(1, i);
        float z = points(2, i);
        proto.add_points(x);
        proto.add_points(y);
        proto.add_points(z);
    }

    return proto;
}

SensorData::LidarData fromProto(const proto::LidarData& proto)
{
    UASSERT(proto.points_size() % 3 == 0);
    int numPoints = proto.points_size() / 3;

    Transform toSensor = fromProto(proto.to_sensor());

    Eigen::Matrix3Xf points(3, numPoints);
    auto pointsIt = proto.points().begin();
    int i = 0;
    while (pointsIt != proto.points().end())
    {
        float x = *pointsIt++;
        float y = *pointsIt++;
        float z = *pointsIt++;
        points(0, i) = x;
        points(1, i) = y;
        points(2, i) = z;
        i++;
    }

    SensorData::LidarData lidarData{std::move(toSensor), std::move(points)};
    return lidarData;
}

proto::SensorData toProto(const SensorData& sensorData)
{
    proto::SensorData proto;
    for (const SensorData::CameraData& cameraData : sensorData.camerasData())
    {
        *proto.add_cameras_data() = toProto(cameraData);
    }
    for (const SensorData::LidarData& lidarData : sensorData.lidarsData())
    {
        *proto.add_lidars_data() = toProto(lidarData);
    }
    return proto;
}

SensorData fromProto(const proto::SensorData& proto)
{
    SensorData sensorData;
    for (const proto::CameraData& cameraDataProto : proto.cameras_data())
    {
        sensorData.addCameraData(fromProto(cameraDataProto));
    }
    for (const proto::LidarData& lidarDataProto : proto.lidars_data())
    {
        sensorData.addLidarData(fromProto(lidarDataProto));
    }
    return sensorData;
}

}