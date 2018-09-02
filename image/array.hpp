#pragma once

#include <algorithm>
#include <array>
#include <range/v3/view.hpp>
#include <type_traits>

/**================================================== *
 * ==========  Array Object  ========== *
 * ================================================== */
namespace common::collection {
template <typename value_type, size_t dimension>
struct ArrayObject {
  using internal_type = std::array<value_type, dimension>;
  using internal_type_ref = std::array<value_type, dimension>&;
  using internal_point_type = value_type;

  ArrayObject() {}
  internal_type value;
  internal_point_type& operator[](const size_t index) noexcept {
    // static_assert(index<dimension, "The index has to be less than the
    // dimension");
    return value[index];
  }

  constexpr const internal_point_type operator[](const size_t index) const
      noexcept {
    static_assert(index < dimension,
                  "The index has to be less than the dimension");
    return value[index];
  }

  using type = ArrayObject<value_type, dimension>;

  constexpr void fill(value_type constant) { value.fill(constant); }

  template <size_t constant>
  constexpr void fill() {
    value.fill(constant);
  }

  auto begin() { return value.begin(); }
  auto end() { return value.end(); }
  template <size_t start, size_t distance>
  auto subArray() {
    ArrayObject<value_type, distance - start> ret_array;
    std::copy_n(value.begin() + start, distance, ret_array.value.begin());
    return ret_array;
  }

  template <size_t distance>
  constexpr auto head() {
    return subArray<0, distance>();
  }
  template <size_t distance>
  constexpr auto tail() {
    return subArray<dimension - distance, dimension>();
  }

  bool operator==(const type& comp) {
    return std::equal(comp.value.begin(), comp.value.end(), value.begin());
  }
  type operator-(const type& comp) {
    type arr;
    std::set_difference(value.begin(), value.end(), comp.value.begin(),
                        comp.value.end(), arr.value.begin());
    // auto v = ranges::view::set_difference(value, comp.value);
    // ranges::copy(v, arr.value);
    return arr;
  }

  type operator-(const size_t& val) {
    type arr;
    std::transform(value.begin(), value.end(), arr.value.begin(),
                   [&](value_type& curr) { return curr - val; });
    return arr;
  }

  type operator+(const type& comp) {
    type arr;
    std::transform(value.begin(), value.end(), comp.value.begin(),
                   arr.value.begin(), std::plus<internal_point_type>());
    // arr.value = value + comp.value;
    return arr;
  }
};

/* =======  End of Array Object  ======= */

/**================================================== *
 * ==========  Ostream for ArrayObject  ========== *
 * ================================================== */
template <size_t dimension, typename value_type>
std::ostream& operator<<(std::ostream& os,
                         const ArrayObject<value_type, dimension>& arr) {
  if (!arr.value.empty()) {
    os << '[';
    for (auto val : arr.value) {
      os << val << ",";
    }
    os << ']' << '\n';
  }
  return os;
}
}  // namespace common::collection

namespace common::collection::detail {
// These are extremely class dependent and should be changed
template <class T, class B>
struct isSameArrayType {
  static const bool value = std::is_same<T, B>::value;
  using type = bool;
};
}  // namespace common::collection::detail

/* =======  End of Ostream for ArrayObject  ======= */
