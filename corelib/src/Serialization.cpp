#include <rtabmap/core/Serialization.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/utilite/ULogger.h>

#include <algorithm>

namespace rtabmap {

const std::string serializationExtension = ".ocp";
static constexpr uint64_t magicNumber = 0x8eed8e9c3d064b79;

Serialization::Serialization(const std::string& fileName)
{
    UASSERT(fileName.size() >= serializationExtension.size() &&
        std::equal(serializationExtension.rbegin(), serializationExtension.rend(),
            fileName.rbegin()));
    output_.open(fileName, std::ios::out | std::ios::binary);
    UASSERT(output_.is_open());
    output_.write((const char*)&magicNumber, sizeof(magicNumber));
}

void Serialization::write(const google::protobuf::Message& proto)
{
    std::string uncompressed;
    proto.SerializeToString(&uncompressed);
    std::string compressed = compressString(uncompressed);
    size_t size = compressed.size();
    output_.write((const char*)&size, sizeof(size));
    output_.write(compressed.data(), size);
}

void Serialization::close()
{
    output_.close();
    UASSERT(!output_.fail());
}

Deserialization::Deserialization(const std::string& fileName)
{
    UASSERT(fileName.size() >= serializationExtension.size() &&
        std::equal(serializationExtension.rbegin(), serializationExtension.rend(),
            fileName.rbegin()));
    input_.open(fileName, std::ios::in | std::ios::binary);
    UASSERT(input_.is_open());
    uint64_t checkMagicNumber;
    input_.read((char*)&checkMagicNumber, sizeof(checkMagicNumber));
    UASSERT(input_.gcount() == sizeof(checkMagicNumber));
    UASSERT(checkMagicNumber == magicNumber);
    readMetaData();
}

std::string Deserialization::readString()
{
    size_t size;
    input_.read((char*)&size, sizeof(size));
    UASSERT(input_.gcount() == sizeof(size));
    std::string compressed(size, '\0');
    input_.read(compressed.data(), size);
    UASSERT(input_.gcount() == size);
    return decompressString(compressed);
}

void Deserialization::readMetaData()
{
    metaData_.ParseFromString(readString());
}

const proto::OccupancyGridMap::MetaData& Deserialization::metaData()
{
    return metaData_;
}

std::optional<proto::OccupancyGridMap::Node> Deserialization::readNode()
{
    if (input_.peek() == EOF)
    {
        return std::nullopt;
    }
    proto::OccupancyGridMap::Node node;
    node.ParseFromString(readString());
    return node;
}

void Deserialization::close()
{
    input_.close();
    UASSERT(!input_.fail());
}

}
