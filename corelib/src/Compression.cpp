#include <rtabmap/core/Compression.h>

#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_stream.hpp>

namespace rtabmap {

std::string compress(const std::string& uncompressed)
{
    std::string compressed;
    boost::iostreams::filtering_ostream out;
    out.push(boost::iostreams::gzip_compressor(
        boost::iostreams::zlib::best_speed));
    out.push(boost::iostreams::back_inserter(compressed));
    boost::iostreams::write(out,
        reinterpret_cast<const char*>(uncompressed.data()),
        uncompressed.size());
    return compressed;
}

std::string decompress(const std::string& compressed)
{
    std::string decompressed;
    boost::iostreams::filtering_ostream out;
    out.push(boost::iostreams::gzip_decompressor());
    out.push(boost::iostreams::back_inserter(decompressed));
    boost::iostreams::write(out,
        reinterpret_cast<const char*>(compressed.data()),
        compressed.size());
    return decompressed;
}

}
