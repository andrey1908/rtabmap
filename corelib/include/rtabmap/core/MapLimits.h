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
        for (int i = 0; i < Dims; i++)
        {
            min_[i] = std::min(min_[i], point[i]);
            max_[i] = std::max(max_[i], point[i]);
        }
    }
    std::array<T, Dims> shape() const
    {
        constexpr bool integral = std::is_integral<T>::value;
        constexpr bool floating = std::is_floating_point<T>::value;
        static_assert(integral || floating);

        std::array<T, Dims> limitsShape;
        for (int i = 0; i < Dims; i++)
        {
            if constexpr(integral)
            {
                limitsShape[i] = max_[i] - min_[i] + static_cast<T>(1);
            }
            if constexpr(floating)
            {
                limitsShape[i] = max_[i] - min_[i];
            }
        }
        return limitsShape;
    }
    static MapLimits<T, Dims> unite(const MapLimits<T, Dims>& a, const MapLimits<T, Dims>& b)
    {
        MapLimits<T, Dims> res;
        for (int i = 0; i < Dims; i++)
        {
            res.min_[i] = std::min(a.min_[i], b.min_[i]);
            res.max_[i] = std::max(a.max_[i], b.max_[i]);
        }
        return res;
    }
    static MapLimits<T, Dims> intersect(const MapLimits<T, Dims>& a, const MapLimits<T, Dims>& b)
    {
        MapLimits<T, Dims> res;
        for (int i = 0; i < Dims; i++)
        {
            res.min_[i] = std::max(a.min_[i], b.min_[i]);
            res.max_[i] = std::min(a.max_[i], b.max_[i]);
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

        for (int i = 0; i < Dims; i++)
        {
            bool checkResult;
            if constexpr(integral)
            {
                checkResult = (min[i] <= max[i] + static_cast<T>(1));
            }
            if constexpr(floating)
            {
                checkResult = (min[i] <= max[i]);
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

        for (int i = 0; i < Dims; i++)
        {
            if constexpr(integral)
            {
                min_[i] = std::min(min_[i], max_[i] + static_cast<T>(1));
            }
            if constexpr(floating)
            {
                min_[i] = std::min(min_[i], max_[i]);
            }
        }
    }

private:
    std::array<T, Dims> min_;
    std::array<T, Dims> max_;
};

typedef MapLimits<int, 2> MapLimitsI;
typedef MapLimits<float, 2> MapLimitsF;

template<typename T, int Dims>
proto::MapLimitsI toProto(const MapLimits<T, Dims>& limits);
template<typename T, int Dims>
MapLimits<T, Dims> fromProto(const proto::MapLimitsI& proto);

}