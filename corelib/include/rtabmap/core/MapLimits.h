#pragma once

#include <limits>
#include <type_traits>
#include <algorithm>
#include <array>

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/proto/MapLimits.pb.h>

namespace rtabmap {

// for integral limits the max limit is included
template<typename T, int Dims>
class MapLimits
{
public:
    MapLimits() {
        std::fill(min_.begin(), min_.end(), std::numeric_limits<T>::max());
        std::fill(max_.begin(), max_.end(), std::numeric_limits<T>::lowest());
    }
    MapLimits(const std::array<T, Dims>& min, const std::array<T, Dims>& max)
    {
        set(min, max);
    }
    void set(const std::array<T, Dims>& min, const std::array<T, Dims>& max)
    {
        UASSERT(checkLimits(min, max));
        min_ = min;
        max_ = max;
    }
    const std::array<T, Dims>& min() const
    {
        return min_;
    }
    const std::array<T, Dims>& max() const
    {
        return max_;
    }
    bool operator==(const MapLimits<T, Dims>& other) const
    {
        return
            std::equal(min_.begin(), min_.end(), other.min_.begin()) &&
            std::equal(max_.begin(), max_.end(), other.max_.begin());
    }
    bool operator!=(const MapLimits<T, Dims>& other) const
    {
        return !operator==(other);
    }
    bool valid() const
    {
        return min_[0] != std::numeric_limits<T>::max();
    }
    void update(const std::array<T, Dims>& point)
    {
        for (int d = 0; d < Dims; d++)
        {
            min_[d] = std::min(min_[d], point[d]);
            max_[d] = std::max(max_[d], point[d]);
        }
    }
    std::array<T, Dims> shape() const
    {
        constexpr bool integral = std::is_integral<T>::value;
        constexpr bool floating = std::is_floating_point<T>::value;
        static_assert(integral || floating);

        std::array<T, Dims> limitsShape;
        for (int d = 0; d < Dims; d++)
        {
            if constexpr(integral)
            {
                limitsShape[d] = max_[d] - min_[d] + static_cast<T>(1);
            }
            if constexpr(floating)
            {
                limitsShape[d] = max_[d] - min_[d];
            }
        }
        return limitsShape;
    }
    static MapLimits<T, Dims> unite(const MapLimits<T, Dims>& a, const MapLimits<T, Dims>& b)
    {
        MapLimits<T, Dims> res;
        for (int d = 0; d < Dims; d++)
        {
            res.min_[d] = std::min(a.min_[d], b.min_[d]);
            res.max_[d] = std::max(a.max_[d], b.max_[d]);
        }
        return res;
    }
    static MapLimits<T, Dims> intersect(const MapLimits<T, Dims>& a, const MapLimits<T, Dims>& b)
    {
        MapLimits<T, Dims> res;
        for (int d = 0; d < Dims; d++)
        {
            res.min_[d] = std::max(a.min_[d], b.min_[d]);
            res.max_[d] = std::min(a.max_[d], b.max_[d]);
        }
        res.normalize();
        return res;
    }

private:
    static bool checkLimits(const std::array<T, Dims>& min, const std::array<T, Dims>& max)
    {
        constexpr bool integral = std::is_integral<T>::value;
        constexpr bool floating = std::is_floating_point<T>::value;
        static_assert(integral || floating);

        for (int d = 0; d < Dims; d++)
        {
            bool checkResult;
            if constexpr(integral)
            {
                checkResult = (min[d] <= max[d] + static_cast<T>(1));
            }
            if constexpr(floating)
            {
                checkResult = (min[d] <= max[d]);
            }
            if (!checkResult)
            {
                return false;
            }
        }
        return true;
    }
    void normalize()
    {
        if (!valid())
        {
            return;
        }
        constexpr bool integral = std::is_integral<T>::value;
        constexpr bool floating = std::is_floating_point<T>::value;
        static_assert(integral || floating);

        for (int d = 0; d < Dims; d++)
        {
            if constexpr(integral)
            {
                min_[d] = std::min(min_[d], max_[d] + static_cast<T>(1));
            }
            if constexpr(floating)
            {
                min_[d] = std::min(min_[d], max_[d]);
            }
        }
    }

private:
    std::array<T, Dims> min_;
    std::array<T, Dims> max_;
};

typedef MapLimits<int, 2> MapLimitsI;
typedef MapLimits<float, 2> MapLimitsF;
typedef MapLimits<int, 3> MapLimitsI3d;
typedef MapLimits<float, 3> MapLimitsF3d;

template<typename T, int Dims>
proto::MapLimitsI toProto(const MapLimits<T, Dims>& limits);
template<typename T, int Dims>
MapLimits<T, Dims> fromProto(const proto::MapLimitsI& proto);

}