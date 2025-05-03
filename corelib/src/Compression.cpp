#include <rtabmap/core/Compression.h>
#include <rtabmap/utilite/ULogger.h>

#include <algorithm>

#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_stream.hpp>

namespace rtabmap {

std::string compressString(const std::string& uncompressed)
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

std::string decompressString(const std::string& compressed)
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

template<typename T>
static void writeToString(std::string& str, const T& value)
{
    const char* valuePtr = (const char*)&value;
    std::copy(valuePtr, valuePtr + sizeof(T), std::back_inserter(str));
}

static void writeToString(std::string& str, const char* value, size_t num)
{
    std::copy(value, value + num, std::back_inserter(str));
}

template<typename T>
static size_t readFromString(const std::string& str, size_t index, T& value)
{
    UASSERT(index + sizeof(T) <= str.size());
    char* valuePtr = (char*)&value;
    auto copyFrom = str.begin() + index;
    std::copy(copyFrom, copyFrom + sizeof(T), valuePtr);
    return index + sizeof(T);
}

static size_t readFromString(const std::string& str, size_t index, char* value, size_t num)
{
    UASSERT(index + num <= str.size());
    auto copyFrom = str.begin() + index;
    std::copy(copyFrom, copyFrom + num, value);
    return index + num;
}

template <typename T, std::size_t Dims>
std::string compressMultiArray(const MultiArray<T, Dims>& array)
{
    UASSERT(array.size() > 0);

    std::string compressed;
    for (int d = 0; d < Dims; d++)
    {
        writeToString(compressed, (int)array.shape()[d]);  // cast to int for backward compatibility
    }
    writeToString(compressed, (int)0);  // backward compatibility; this was a type of opencv matrix

    T currentValue;
    std::uint16_t counter = 0;
    for (auto it = array.begin(); it != array.end(); ++it)
    {
        const T& value = *it;
        if (value != currentValue)
        {
            if (counter > 0)
            {
                writeToString(compressed, counter);
                writeToString(compressed, currentValue);
                counter = 0;
            }
            currentValue = value;
        }
        counter++;
    }
    UASSERT(counter > 0);
    writeToString(compressed, counter);
    writeToString(compressed, currentValue);

    return compressed;
}

template <typename T, std::size_t Dims>
MultiArray<T, Dims> decompressMultiArray(const std::string& compressed)
{
    size_t index = 0;
    std::array<std::size_t, Dims> shape;
    for (int d = 0; d < Dims; d++)
    {
        int axisSize;
        index = readFromString(compressed, index, axisSize);
        shape[d] = axisSize;
    }
    index += sizeof(int);  // backward compatibility; this was a type of opencv matrix

    MultiArray<T, Dims> array(shape);
    T currentValue;
    std::uint16_t counter = 0;
    constexpr std::uint16_t newRow = 0xFFFF;  // backward compatibility; not used in new versions
    for (auto it = array.begin(); it != array.end(); ++it)
    {
        if (counter == 0)
        {
            index = readFromString(compressed, index, counter);
            if (counter == newRow)
            {
                index = readFromString(compressed, index, counter);
            }
            UASSERT(counter != newRow);
            index = readFromString(compressed, index, currentValue);
        }
        *it = currentValue;
        counter--;
    }
    UASSERT(counter == 0);
    UASSERT(index = compressed.size());

    return array;
}

std::string compressMat(const cv::Mat& mat)
{
    UASSERT(mat.total() > 0);

    std::string compressed;
    writeToString(compressed, mat.rows);
    writeToString(compressed, mat.cols);
    writeToString(compressed, mat.type());

    std::vector<char> currentValue(mat.elemSize());
    std::uint16_t counter = 0;
    constexpr std::uint16_t newRow = 0xFFFF;
    for (int y = 0; y < mat.rows; y++)
    {
        for (int x = 0; x < mat.cols; x++)
        {
            const char* valuePtr = mat.ptr<const char>(y, x);
            if (std::equal(currentValue.begin(), currentValue.end(), valuePtr))
            {
                counter++;
            }
            else
            {
                if (counter > 0)
                {
                    writeToString(compressed, counter);
                    writeToString(compressed, currentValue.data(), currentValue.size());
                    counter = 0;
                }
                std::copy(valuePtr, valuePtr + mat.elemSize(), currentValue.begin());
                counter++;
            }
        }
        UASSERT(counter > 0);
        writeToString(compressed, counter);
        writeToString(compressed, currentValue.data(), currentValue.size());
        writeToString(compressed, newRow);
        counter = 0;
    }
    return compressed;
}

cv::Mat decompressMat(const std::string& compressed)
{
    size_t index = 0;
    int rows, cols, type;
    index = readFromString(compressed, index, rows);
    index = readFromString(compressed, index, cols);
    index = readFromString(compressed, index, type);

    cv::Mat mat = cv::Mat(rows, cols, type);
    std::uint16_t number;
    std::vector<char> currentValue(mat.elemSize());
    constexpr std::uint16_t newRow = 0xFFFF;
    int currentRow = 0;
    int currentCol = 0;
    while (index != compressed.size())
    {
        index = readFromString(compressed, index, number);
        if (number == newRow)
        {
            currentRow++;
            currentCol = 0;
            continue;
        }
        index = readFromString(compressed, index, currentValue.data(), currentValue.size());
        for (int col = currentCol; col < currentCol + (int)number; col++)
        {
            UASSERT(col < mat.cols);
            std::copy(currentValue.begin(), currentValue.end(),
                mat.ptr<char>(currentRow, col));
        }
        currentCol += (int)number;
    }
    UASSERT(currentRow == mat.rows && currentCol == 0);
    return mat;
}

template std::string compressMultiArray(const MultiArray<std::uint8_t, 2>& array);
template std::string compressMultiArray(const MultiArray<std::uint8_t, 3>& array);
template std::string compressMultiArray(const MultiArray<std::int32_t, 2>& array);
template std::string compressMultiArray(const MultiArray<std::int32_t, 3>& array);

template MultiArray<std::uint8_t, 2> decompressMultiArray<std::uint8_t, 2>(const std::string& compressed);
template MultiArray<std::uint8_t, 3> decompressMultiArray<std::uint8_t, 3>(const std::string& compressed);
template MultiArray<std::int32_t, 2> decompressMultiArray<std::int32_t, 2>(const std::string& compressed);
template MultiArray<std::int32_t, 3> decompressMultiArray<std::int32_t, 3>(const std::string& compressed);

}
