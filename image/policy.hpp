#pragma once
#include <algorithm>
#include <array>
#include <iostream>
#include <range/v3/view.hpp>
#include <type_traits>
#include "array.hpp"

namespace common::image{
/**================================================== *
 * ==========  Image Policy  ========== *
 * ================================================== */

template <size_t Dimension>
class ImagePolicy {
 public:
  using OffsetType = u_long;
  using IndexType = common::collection::ArrayObject<OffsetType, Dimension>;
  using SizeType = common::collection::ArrayObject<OffsetType, Dimension>;
  using BoundingBoxType = common::collection::ArrayObject<OffsetType, Dimension * 2>;
  using OffsetTableType = collection::ArrayObject<OffsetType, Dimension + 1>;
  const int dimension = Dimension;

  // ImagePolicy(SizeType s) : size(s) {
  //   offset_table.fill(1);
  //   computeOffsetTable<0>();
  // }
  ImagePolicy() {}

  constexpr auto convertOffsetToIndex(OffsetTableType& offset_table,
                                      OffsetType& offset) {
    IndexType index{};
    toIndexHelper<Dimension>(offset_table, index, offset);
    return index;
  }
  constexpr auto convertIndexToOffset(OffsetTableType& offset_table,
                                      IndexType& index) {
    OffsetType offset = 0;
    toOffsetHelper<0>(offset_table, index, offset);
    return offset;
  }

  constexpr auto convertSizeToBoundingBox(IndexType& index, SizeType size) {
    BoundingBoxType bbox{};
    toBoundingBoxFromSizeHelper<0>(index, size, bbox);
    return bbox;
  }

  constexpr auto convertBoundingBoxToSize(BoundingBoxType bbox) {
    SizeType size{};
    toSizeFromBoundingBoxHelper<0>(size, bbox);
    return size;
  }

  constexpr auto computeOffsetTable(SizeType size) {
    OffsetTableType offset_table{};
    offset_table.fill(1);
    return computeOffsetTableHelper<0>(size, offset_table);
  }

  /**================================================== *
   * ==========  PROTECTED  ========== *
   * ================================================== */

 protected:
  // template <OffsetType index>
  // constexpr auto getOffset() {}

  // constexpr SizeType getSize() const{ return size; }

  // void setSizeFromBoundingBox(BoundingBoxType& bounding_box) {
  //   // std::cout << bounding_box << '\n';
  //   // size.fill(0);
  //   toSizeFromBoundingBoxHelper<0>(bounding_box);
  //   // std::cout << size << '\n';

  //   // computeOffsetTable<0>();
  //   // std::cout << offset_table << '\n';
  // }

  /*--------  Variables  --------*/

  // OffsetTableType offset_table;

  // SizeType size;

  /* =======  End of PROTECTED  ======= */

  /**================================================== *
   * ==========  PRIVATE  ========== *
   * ================================================== */

 private:
  /**
   *
   * Compute offset table
   *
   */

  /**
   *
   * Helper for Index
   *
   */

  template <size_t loop>
  void toIndexHelper(OffsetTableType& offset_table,
                     IndexType& index,
                     OffsetType offset) {
    if constexpr (loop != 0) {
      index[loop] = (offset / offset_table[loop]);
      offset = offset - (index[loop] * offset_table[loop]);
      toIndexHelper<loop - 1>(offset_table, index, offset);
    } else {
      index[loop] = offset;
    }
  }
  /**
   *
   * Helper for offset
   *
   */

  template <size_t loop>
  void toOffsetHelper(OffsetTableType& offset_table,
                      IndexType& index,
                      OffsetType& offset) {
    if constexpr (loop != Dimension) {
      offset = offset + (index[loop] * offset_table[loop]);
      toOffsetHelper<loop + 1>(offset_table, index, offset);
    }
  }

  template <size_t loop>
  auto toSizeFromBoundingBoxHelper(SizeType& size,

                                   BoundingBoxType& bbox) {
    if constexpr (loop != Dimension) {
      size[loop] = bbox[loop + Dimension] - bbox[loop];
      toSizeFromBoundingBoxHelper<loop + 1>(size, bbox);
    }
  }

  template <size_t loop>
  auto toBoundingBoxFromSizeHelper(IndexType& start_index,
                                   SizeType& size,

                                   BoundingBoxType& bbox) {
    if constexpr (loop != Dimension) {
      bbox[loop] = start_index[loop];
      bbox[loop + dimension] = start_index[loop] + size[loop];
      toBoundingBoxFromSizeHelper<loop + 1>(start_index, size, bbox);
    }
  }

  template <size_t loop>
  constexpr auto computeOffsetTableHelper(SizeType& size,
                                          OffsetTableType& offset_table) {
    if constexpr (loop != (Dimension + 1)) {
      offset_table[loop + 1] = offset_table[loop] * size[loop];
      computeOffsetTableHelper<loop + 1>(size, offset_table);
    }
    return offset_table;
  }

  /* =======  End of PRIVATE  ======= */
};

/* =======  End of Image Policy  ======= */

}
