#pragma once

#include <iostream>
#include <memory>
#include "policy.hpp"

namespace common::collection {
/**================================================== *
 * ==========  PointType  ========== *
 * ================================================== */
template <size_t dimension,
          class ValueType,
          class OISPolicy = image::ImagePolicy<dimension>>
struct Point {
  using IndexType = typename OISPolicy::IndexType;
  using OffsetType = typename OISPolicy::OffsetType;
  using PointerType = std::unique_ptr<Point>;

  Point(ValueType i_value, IndexType i_index, OffsetType i_offset)
      : index(i_index), value(i_value), offset(i_offset) {}

  IndexType index;
  OffsetType offset;
  ValueType value;
};

template <size_t dimension, typename value_type>
std::ostream& operator<<(std::ostream& os,
                         const Point<dimension, value_type>& point) {
  os << "Offset: " << point.offset << " Value: " << point.value
     << " Index: " << point.index;
  return os;
}
/* =======  End of PointType  ======= */
}  // namespace common::collection
