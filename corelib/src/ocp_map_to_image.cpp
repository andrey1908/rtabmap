#include <iostream>
#include <memory>
#include <optional>
#include <set>
#include <map>

#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>

#include <rtabmap/core/Serialization.h>
#include <rtabmap/core/OccupancyGridMap.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/core/Time.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Trajectory.h>
#include <rtabmap/core/LaserScan.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Node.h>
#include <rtabmap/core/LocalMap.h>
#include <rtabmap/core/MapLimits.h>
#include <rtabmap/utilite/ULogger.h>

#include <rtabmap/proto/RawData.pb.h>

#include <kas_utils/yaml_utils.h>

namespace po = boost::program_options;
using kas_utils::mergeYaml;

namespace rtabmap {

cv::Mat drawGrid(const OccupancyGrid::GridType& grid, int scale)
{
    cv::Mat gridImage(grid.rows() * scale, grid.cols() * scale, CV_8UC3);
    for (int y = 0; y < grid.rows(); y++)
    {
        for (int x = 0; x < grid.cols(); x++)
        {
            char cellValue = grid.coeff(y, x);
            cv::Scalar color;
            switch (cellValue)
            {
            case (char)0:  // free
                color[0] = 255;
                color[1] = 255;
                color[2] = 255;
                break;
            case (char)100:  // occupied
                color[0] = 0;
                color[1] = 0;
                color[2] = 0;
                break;
            case (char)-1:  // unknown
                color[0] = 100;
                color[1] = 100;
                color[2] = 100;
                break;
            default:
                UASSERT(false);
            }

            cv::Point pt1(x * scale, y * scale);
            cv::Point pt2((x + 1) * scale - 1, (y + 1) * scale - 1);
            cv::rectangle(gridImage, pt1, pt2, color, -1);
        }
    }

    cv::Mat gridImageFlipped;
    cv::flip(gridImage, gridImageFlipped, 0);

    return gridImageFlipped;
}

void ocpMapToImage(
    const std::vector<std::string>& configFiles, const std::string& ocpFile,
    const std::string& outImageFile, int scale)
{
    YAML::Node config;
    for (const std::string& configFile : configFiles)
    {
        YAML::Node updateConfig = YAML::LoadFile(configFile);
        bool ret = mergeYaml(config, updateConfig);
        UASSERT(ret);
    }
    OccupancyGridMap::Parameters parameters =
        OccupancyGridMap::Parameters::createParameters(config["OccupancyGridMap"]);
    OccupancyGridMap gridMap(parameters);

    gridMap.load(ocpFile);
    OccupancyGrid grid = gridMap.getOccupancyGrid(0);
    cv::Mat gridImage = drawGrid(grid.grid, scale);

    cv::imwrite(outImageFile, gridImage);
}

}


int main(int argc, char** argv)
{
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("config-files", po::value<std::vector<std::string>>())
        ("ocp-file", po::value<std::string>())
        ("out-image-file", po::value<std::string>())
        ("scale", po::value<int>()->default_value(1));

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << "\n";
        return 0;
    }

    UASSERT(vm.count("config-files"));
    UASSERT(vm.count("ocp-file"));
    UASSERT(vm.count("out-image-file"));
    UASSERT(vm.count("scale"));
    UASSERT(vm["scale"].as<int>() > 0);

    rtabmap::ocpMapToImage(
        vm["config-files"].as<std::vector<std::string>>(),
        vm["ocp-file"].as<std::string>(),
        vm["out-image-file"].as<std::string>(),
        vm["scale"].as<int>());
}

