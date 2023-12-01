#pragma once

#include <optional>
#include <string>
#include <fstream>
#include <cinttypes>

#include <google/protobuf/message.h>
#include <rtabmap/proto/OccupancyGridMap.pb.h>
#include <rtabmap/proto/RawData.pb.h>

namespace rtabmap {

enum MapVersions
{
    mapLatestVersion = 0
};

class MapSerialization
{
public:
    MapSerialization(const std::string& fileName, float cellSize);

    void write(const proto::OccupancyGridMap::Node& proto);
    void close();

private:
    void writeMetaData(float cellSize);
    void writeString(const std::string& uncompressed);

private:
    std::ofstream output_;
};

class MapDeserialization
{
public:
    MapDeserialization(const std::string& fileName);

    const proto::OccupancyGridMap::MetaData& metaData();

    std::optional<proto::OccupancyGridMap::Node> read();
    void close();

private:
    void readMetaData();
    std::string readString();

private:
    std::ifstream input_;
    proto::OccupancyGridMap::MetaData metaData_;
};

enum RawDataVersions
{
    rawDataLatestVersion = 0
};

class RawDataSerialization
{
public:
    RawDataSerialization(const std::string& fileName);

    void write(const proto::RawData& rawData);
    void close();

private:
    void writeMetaData();
    void writeString(const std::string& uncompressed);

private:
    std::ofstream output_;
};

class RawDataDeserialization
{
public:
    RawDataDeserialization(const std::string& fileName);

    const proto::RawData::MetaData& metaData();

    std::optional<proto::RawData> read();
    void close();

private:
    void readMetaData();
    std::string readString();

private:
    std::ifstream input_;
    proto::RawData::MetaData metaData_;
};

}
