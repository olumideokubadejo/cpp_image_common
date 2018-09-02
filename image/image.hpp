#pragma once
#include <iostream>
#include <memory>
#include <mutex>
#include <type_traits>
#include "point.hpp"
#include "policy.hpp"

using namespace common::collection;
namespace common::image::details {
template <typename InputType, typename ValueType, typename PointType>
struct isPointConstructible {
  static constexpr bool value =
      std::disjunction<std::is_same<InputType, ValueType>,
                       std::is_same<InputType, PointType>>::value;
  using type = bool;
};
}  // namespace common::image::details
namespace common::image {
template <size_t Dimension,
          class ValueType,
          class OISPolicyType = ImagePolicy<Dimension>,
          class PointType = Point<Dimension, ValueType>>

class Image : public OISPolicyType {
 public:
  using SizeType = typename OISPolicyType::SizeType;
  using IndexType = typename OISPolicyType::IndexType;
  using OffsetType = typename OISPolicyType::OffsetType;
  using OffsetTableType = typename OISPolicyType::OffsetTableType;
  using OISPolicyType ::OISPolicyType;

  static const auto dimension = Dimension;
  using PixelValueType = ValueType;

  Image(ValueType* i_data, SizeType i_size) : size(i_size), OISPolicyType() {
    offset_table = OISPolicyType::computeOffsetTable(size);
    m_data = i_data;
  }

  Image(ValueType* i_data) { m_data = i_data; }

  template <typename... Functions>
  void cascadeNeighborhoodFunctions(Functions... functions) {}

  template <typename Type, typename ReturnType = ValueType>
  constexpr auto getValue(Type location) {
    std::lock_guard<std::mutex> guard(data_mutex);
    if constexpr (std::is_same<Type, OffsetType>::value) {
      return getPoint<ReturnType>(location);
    } else if constexpr (std::is_same<Type, IndexType>::value) {
      return getPoint<ReturnType>(
          OISPolicyType::convertIndexToOffset(offset_table, location));
    }
  }
  auto getOffsetTable() const { return offset_table; }

  auto getBoundingBox() {
    IndexType index{};
    index.fill(0);
    return OISPolicyType::convertSizeToBoundingBox(index, size);
  }

 private:
  ValueType* m_data;
  SizeType size;
  OffsetTableType offset_table;
  std::mutex data_mutex;

  template <class ReturnType>
  constexpr auto getPoint(OffsetType offset) {
    std::lock_guard<std::mutex> guard(data_mutex);
    if constexpr (std::is_same<ReturnType, ValueType>::value) {
      return (*(m_data + offset));
    } else {
      return std::make_unique<PointType>(
          *(m_data + offset),
          OISPolicyType::convertOffsetToIndex(offset_table, offset), offset);
    }
  }
};
}  // namespace common::image
