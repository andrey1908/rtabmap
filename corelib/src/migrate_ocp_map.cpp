#include <iostream>
#include <memory>
#include <optional>

#include <boost/program_options.hpp>
#include <opencv2/core/core.hpp>

#include <rtabmap/core/LocalMap.h>
#include <rtabmap/core/MapLimits.h>
#include <rtabmap/core/Trajectory.h>
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

void replaceSensorBlindRangeWithMaybeEmptyCells(proto::OccupancyGridMap::Node& proto)
{
    if (proto.local_map().sensor_blind_range_2d_sqr() == 0.0f)
    {
        return;
    }

    cv::Mat grid = decompressMat(proto.local_map().colored_grid().grid_compressed());
    UASSERT(grid.type() == CV_8UC1);
    MapLimitsI limits = fromProto<int, 2>(proto.local_map().colored_grid().limits());
    Transform toSensor = fromProto(proto.local_map().to_sensor());
    float cellSize = proto.local_map().colored_grid().cell_size();
    for (int y = 0; y < grid.rows; y++)
    {
        for (int x = 0; x < grid.cols; x++)
        {
            std::uint8_t& value = grid.at<std::uint8_t>(y, x);
            if (value == LocalMap::ColoredGrid::emptyCellValue)
            {
                float xf = (x + limits.min()[0] + 0.5f) * cellSize;
                float yf = (y + limits.min()[1] + 0.5f) * cellSize;
                float xs = xf - toSensor.translation().x();
                float ys = yf - toSensor.translation().y();
                if (xs * xs + ys * ys <= proto.local_map().sensor_blind_range_2d_sqr())
                {
                    value = LocalMap::ColoredGrid::maybeEmptyCellValue;
                }
            }
        }
    }

    proto.mutable_local_map()->mutable_colored_grid()->set_grid_compressed(
        compressMat(grid));
}

Trajectory getLocalPoses(MapDeserialization& reader)
{
    Trajectory localPoses;
    std::optional<proto::OccupancyGridMap::Node> proto;
    while (proto = reader.read())
    {
        if (!proto->has_global_pose())
        {
            continue;
        }
        localPoses.addPose(
            fromProto(proto->local_map().time()), fromProto(proto->global_pose()));
    }
    return localPoses;
}

void migrateOcpMap(const std::string& inOcpFile, const std::string& outOcpFile)
{
    MapDeserialization reader(inOcpFile);

    Trajectory localPoses;
    if (reader.metaData().version() <= MapVersions::mapNoLocalPoses)
    {
        localPoses = getLocalPoses(reader);
        reader.close();
        reader = MapDeserialization(inOcpFile);
    }
    else
    {
        localPoses = fromProto(reader.localPoses());
    }

    MapSerialization writer(outOcpFile, reader.metaData().cell_size(), localPoses);

    std::optional<proto::OccupancyGridMap::Node> proto;
    while (proto = reader.read())
    {
        if (reader.metaData().version() <= MapVersions::mapNoLocalPoses &&
            !proto->has_global_pose())
        {
            continue;
        }

        if (reader.metaData().version() <= MapVersions::mapOldCellValues)
        {
            migrateCellValues(*proto);
        }
        if (reader.metaData().version() <= MapVersions::mapNoMaybeEmptyCells)
        {
            replaceSensorBlindRangeWithMaybeEmptyCells(*proto);
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

