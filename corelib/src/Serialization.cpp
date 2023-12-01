#include <rtabmap/core/Serialization.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/utilite/ULogger.h>

#include <algorithm>

namespace rtabmap {

const std::string mapExtension = ".ocp";
const std::string rawDataExtension = ".rd";
static constexpr uint64_t magicNumber = 0x8eed8e9c3d064b79;

/* MapSerialization */

MapSerialization::MapSerialization(const std::string& fileName, float cellSize)
{
    UASSERT(fileName.size() >= mapExtension.size() &&
        std::equal(mapExtension.rbegin(), mapExtension.rend(),
            fileName.rbegin()));
    output_.open(fileName, std::ios::out | std::ios::binary);
    UASSERT(output_.is_open());
    output_.write((const char*)&magicNumber, sizeof(magicNumber));

    writeMetaData(cellSize);
}

void MapSerialization::writeMetaData(float cellSize)
{
    proto::OccupancyGridMap::MetaData metaData;
    metaData.set_version(MapVersions::mapLatestVersion);
    metaData.set_cell_size(cellSize);

    std::string uncompressed;
    metaData.SerializeToString(&uncompressed);
    writeString(uncompressed);
}

void MapSerialization::write(const proto::OccupancyGridMap::Node& node)
{
    std::string uncompressed;
    node.SerializeToString(&uncompressed);
    writeString(uncompressed);
}

void MapSerialization::writeString(const std::string& uncompressed)
{
    std::string compressed = compressString(uncompressed);
    size_t size = compressed.size();
    output_.write((const char*)&size, sizeof(size));
    output_.write(compressed.data(), size);
}

void MapSerialization::close()
{
    output_.close();
    UASSERT(!output_.fail());
}

/* MapDeserialization */

MapDeserialization::MapDeserialization(const std::string& fileName)
{
    UASSERT(fileName.size() >= mapExtension.size() &&
        std::equal(mapExtension.rbegin(), mapExtension.rend(),
            fileName.rbegin()));
    input_.open(fileName, std::ios::in | std::ios::binary);
    UASSERT(input_.is_open());
    uint64_t checkMagicNumber;
    input_.read((char*)&checkMagicNumber, sizeof(checkMagicNumber));
    UASSERT(input_.gcount() == sizeof(checkMagicNumber));
    UASSERT(checkMagicNumber == magicNumber);

    readMetaData();
}

void MapDeserialization::readMetaData()
{
    metaData_.ParseFromString(readString());
}

std::optional<proto::OccupancyGridMap::Node> MapDeserialization::read()
{
    if (input_.peek() == EOF)
    {
        return std::nullopt;
    }
    proto::OccupancyGridMap::Node node;
    node.ParseFromString(readString());
    return node;
}

std::string MapDeserialization::readString()
{
    size_t size;
    input_.read((char*)&size, sizeof(size));
    UASSERT(input_.gcount() == sizeof(size));
    std::string compressed(size, '\0');
    input_.read(compressed.data(), size);
    UASSERT(input_.gcount() == size);
    return decompressString(compressed);
}

const proto::OccupancyGridMap::MetaData& MapDeserialization::metaData()
{
    return metaData_;
}

void MapDeserialization::close()
{
    input_.close();
    UASSERT(!input_.fail());
}

/* RawDataSerialization */

RawDataSerialization::RawDataSerialization(const std::string& fileName)
{
    UASSERT(fileName.size() >= rawDataExtension.size() &&
        std::equal(rawDataExtension.rbegin(), rawDataExtension.rend(),
            fileName.rbegin()));
    output_.open(fileName, std::ios::out | std::ios::binary);
    UASSERT(output_.is_open());
    output_.write((const char*)&magicNumber, sizeof(magicNumber));

    writeMetaData();
}

void RawDataSerialization::writeMetaData()
{
    proto::RawData::MetaData metaData;
    metaData.set_version(RawDataVersions::rawDataLatestVersion);

    std::string uncompressed;
    metaData.SerializeToString(&uncompressed);
    writeString(uncompressed);
}

void RawDataSerialization::write(const proto::RawData& rawData)
{
    std::string uncompressed;
    rawData.SerializeToString(&uncompressed);
    writeString(uncompressed);
}

void RawDataSerialization::writeString(const std::string& uncompressed)
{
    size_t size = uncompressed.size();
    output_.write((const char*)&size, sizeof(size));
    output_.write(uncompressed.data(), size);
}

void RawDataSerialization::close()
{
    output_.close();
    UASSERT(!output_.fail());
}

/* RawDataDeserialization */

RawDataDeserialization::RawDataDeserialization(const std::string& fileName)
{
    UASSERT(fileName.size() >= rawDataExtension.size() &&
        std::equal(rawDataExtension.rbegin(), rawDataExtension.rend(),
            fileName.rbegin()));
    input_.open(fileName, std::ios::in | std::ios::binary);
    UASSERT(input_.is_open());
    uint64_t checkMagicNumber;
    input_.read((char*)&checkMagicNumber, sizeof(checkMagicNumber));
    UASSERT(input_.gcount() == sizeof(checkMagicNumber));
    UASSERT(checkMagicNumber == magicNumber);

    readMetaData();
}

void RawDataDeserialization::readMetaData()
{
    metaData_.ParseFromString(readString());
}

std::optional<proto::RawData> RawDataDeserialization::read()
{
    if (input_.peek() == EOF)
    {
        return std::nullopt;
    }
    proto::RawData rawData;
    rawData.ParseFromString(readString());
    return rawData;
}

std::string RawDataDeserialization::readString()
{
    size_t size;
    input_.read((char*)&size, sizeof(size));
    UASSERT(input_.gcount() == sizeof(size));
    std::string uncompressed(size, '\0');
    input_.read(uncompressed.data(), size);
    UASSERT(input_.gcount() == size);
    return uncompressed;
}

const proto::RawData::MetaData& RawDataDeserialization::metaData()
{
    return metaData_;
}

void RawDataDeserialization::close()
{
    input_.close();
    UASSERT(!input_.fail());
}

}
