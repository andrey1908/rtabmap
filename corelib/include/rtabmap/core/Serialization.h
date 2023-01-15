#pragma once

#include <optional>
#include <string>
#include <fstream>
#include <cinttypes>

#include <google/protobuf/message.h>
#include <rtabmap/proto/OccupancyGridMap.pb.h>

namespace rtabmap {

extern const std::string serializationExtension;

class Serialization
{
public:
	Serialization(const std::string& fileName);

	void write(const google::protobuf::Message& proto);
	void close();

private:
	std::ofstream output_;
};

class Deserialization
{
public:
	Deserialization(const std::string& fileName);

	const proto::OccupancyGridMap::MetaData& getMetaData();
	std::optional<proto::OccupancyGridMap::Node> readNode();
	void close();

private:
	void readMetaData();
	std::string readString();

private:
	std::ifstream input_;
	proto::OccupancyGridMap::MetaData metaData_;
};

}
