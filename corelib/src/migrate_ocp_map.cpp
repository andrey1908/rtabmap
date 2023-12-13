#include <iostream>
#include <memory>
#include <optional>

#include <boost/program_options.hpp>
#include <opencv2/core/core.hpp>

#include <rtabmap/core/LocalMap.h>
#include <rtabmap/core/Serialization.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/utilite/ULogger.h>


namespace po = boost::program_options;

namespace rtabmap {

void migrateCellValues(proto::OccupancyGridMap::Node& proto)
{
    cv::Mat oldGrid = decompressMat(proto.local_map().colored_grid().grid_compressed());
    UASSERT(oldGrid.type() == CV_8SC1);
    cv::Mat newGrid(oldGrid.size(), CV_8UC1);

    constexpr std::int8_t oldUnknownCellValue = -1;
    constexpr std::int8_t oldEmptyCellValue = 0;
    constexpr std::int8_t oldOccupiedCellValue = 100;

    for (int i = 0; i < oldGrid.rows; i++)
    {
        for (int j = 0; j < oldGrid.cols; j++)
        {
            const std::int8_t& oldValue = oldGrid.at<std::int8_t>(i, j);
            std::uint8_t& newValue = newGrid.at<std::uint8_t>(i, j);
            switch (oldValue)
            {
            case oldUnknownCellValue:
                newValue = LocalMap::ColoredGrid::unknownCellValue;
                break;
            case oldEmptyCellValue:
                newValue = LocalMap::ColoredGrid::emptyCellValue;
                break;
            case oldOccupiedCellValue:
                newValue = LocalMap::ColoredGrid::occupiedCellValue;
                break;
            default:
                UASSERT_MSG(false, "Unknown old cell value. This should not happen.");
            }
        }
    }

    proto.mutable_local_map()->mutable_colored_grid()->set_grid_compressed(
        compressMat(newGrid));
}

void migrateOcpMap(const std::string& inOcpFile, const std::string& outOcpFile)
{
    MapDeserialization reader(inOcpFile);
    MapSerialization writer(outOcpFile, reader.metaData().cell_size());

    std::optional<proto::OccupancyGridMap::Node> proto;
    while (proto = reader.read())
    {
        if (reader.metaData().version() <= MapVersions::mapOldCellValues)
        {
            migrateCellValues(*proto);
        }
        writer.write(*proto);
    }

    writer.close();
    reader.close();
}

}


int main(int argc, char** argv)
{
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("in-ocp-file", po::value<std::string>())
        ("out-ocp-file", po::value<std::string>());

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << "\n";
        return 0;
    }

    UASSERT(vm.count("in-ocp-file"));
    UASSERT(vm.count("out-ocp-file"));

    rtabmap::migrateOcpMap(
        vm["in-ocp-file"].as<std::string>(),
        vm["out-ocp-file"].as<std::string>());
}

