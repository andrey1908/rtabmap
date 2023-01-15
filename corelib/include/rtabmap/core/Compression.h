#pragma once

#include <string>

namespace rtabmap {

std::string compress(const std::string& uncompressed);
std::string decompress(const std::string& compressed);

}
