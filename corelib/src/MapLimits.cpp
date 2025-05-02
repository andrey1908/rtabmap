#include <rtabmap/core/MapLimits.h>

namespace rtabmap {

template <typename T, int Dims>
proto::MapLimitsI toProto(const MapLimits<T, Dims>& limits)
{
    proto::MapLimitsI proto;
    proto.set_is_3d(Dims == 3);
    *proto.mutable_min() = {limits.min().begin(), limits.min().end()};
    *proto.mutable_max() = {limits.max().begin(), limits.max().end()};
    return proto;
}

template <typename T, int Dims>
MapLimits<T, Dims> fromProto(const proto::MapLimitsI& proto)
{
    constexpr bool is_3d = (Dims == 3);
    UASSERT(proto.is_3d() == is_3d);

    // backward compatibility
    if (proto.min_size() == 0)
    {
        MapLimits<T, Dims> limits(
            {proto.min_y(), proto.min_x()},
            {proto.max_y(), proto.max_x()});
        return limits;
    }

    std::array<T, Dims> min;
    std::array<T, Dims> max;
    for (int i = 0; i < Dims; i++)
    {
        min[i] = proto.min(i);
        max[i] = proto.max(i);
    }

    MapLimits<T, Dims> limits(min, max);
    return limits;
}

template proto::MapLimitsI toProto(const MapLimitsI&);
template MapLimitsI fromProto<int, 2>(const proto::MapLimitsI&);

template proto::MapLimitsI toProto(const MapLimitsI3d&);
template MapLimitsI3d fromProto<int, 3>(const proto::MapLimitsI&);

}