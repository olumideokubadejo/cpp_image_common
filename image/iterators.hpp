#pragma once
#include <iostream>
#include <thread>
#include <type_traits>
#include "image.hpp"
#include "point.hpp"
#include "policy.hpp"

using namespace common::image;
using namespace common::collection;
namespace common::iterator {
template <class ReturnType, class OISPolicyType>
struct Function {};
template <size_t dimension, class ImagePolicyType = ImagePolicy<dimension>>
class CursorWithBoundsCheck : public ImagePolicyType {
 public:
  using OISPolicyType = ImagePolicyType;
  using BoundingBoxType = typename OISPolicyType::BoundingBoxType;
  using OffsetType = typename OISPolicyType::OffsetType;
  using IndexType = typename OISPolicyType::IndexType;
  using SizeType = typename OISPolicyType::SizeType;
  using OffsetTableType = typename ImagePolicyType::OffsetTableType;
  BoundingBoxType bounding_box;
  OffsetTableType offset_table;

  CursorWithBoundsCheck(const BoundingBoxType& b_box, OffsetTableType o_table)
      : bounding_box(b_box), offset_table(o_table), OISPolicyType() {
    size = OISPolicyType::convertBoundingBoxToSize(bounding_box);

    internal_index.template fill<0>();
    start_index = bounding_box.template head<dimension>();
    start_offset =
        OISPolicyType::convertIndexToOffset(offset_table, start_index);
    auto end_index = start_index + size;
    end_offset = OISPolicyType::convertIndexToOffset(offset_table, end_index);
  }

  void toNextPoint() {
    current_offset++;
    internal_index[0]++;
    if (internal_index[0] == size[0]) {
      toNextLine();
    }
  }
  void toNextLine() {
    for (size_t index = 0; index < dimension; index++) {
      if ((internal_index[index] == (size[index])) &&
          ((index + 1) < dimension)) {
        internal_index[index + 1] = internal_index[index + 1] + 1;
        internal_index[index] = 0;
      } else if ((index + 1 == dimension) &&
                 (internal_index[index] == size[index])) {
        toEnd();
      }
    }
    auto ind = internal_index + start_index;
    current_offset = OISPolicyType::convertIndexToOffset(offset_table, ind);
  }
  void toPreviousPoint() {
    current_offset--;
    internal_index[0]--;
    if (internal_index[0] < 0) {
      toPreviousLine();
    }
  }

  void toPreviousLine() {
    for (size_t index = 0; index < OISPolicyType::dimension; index++) {
      if ((internal_index[index] < 0) &&
          ((index + 1) < OISPolicyType::dimension)) {
        internal_index[index + 1] = internal_index[index + 1] - 1;
        toLineEnd();
      }
    }
    current_offset =
        OISPolicyType::convertIndexToOffset(internal_index + start_index);
  }
  void toLineBegin() { internal_index[0] = 0; }
  void toLineEnd() { internal_index[0] = OISPolicyType::size[0] - 1; }
  void toBegin() { current_offset = start_offset; }
  void toEnd() {
    current_offset = end_offset;
    internal_index = size;
  }
  bool isEnd() { return current_offset >= end_offset; }

  OffsetType start_offset;
  OffsetType end_offset;
  OffsetType current_offset;
  IndexType internal_index;
  IndexType start_index;
  SizeType size;
};

template <size_t dimension, class ImagePolicyType = ImagePolicy<dimension>>
class CursorWithoutBoundsCheck : public ImagePolicyType {
 public:
  using OISPolicyType = ImagePolicyType;
  using BoundingBoxType = typename OISPolicyType::BoundingBoxType;
  using OffsetType = typename OISPolicyType::OffsetType;
  using IndexType = typename OISPolicyType::IndexType;
  using SizeType = typename OISPolicyType::SizeType;

  using OffsetTableType = typename ImagePolicyType::OffsetTableType;
  BoundingBoxType bounding_box;

  CursorWithoutBoundsCheck(const BoundingBoxType& b_box) : ImagePolicyType() {
    bounding_box = b_box;
    size = ImagePolicyType::convertBoundingBoxToSize(bounding_box);
    offset_table = ImagePolicyType::computeOffsetTable(size);
    start_offset = 0;
    end_offset = offset_table[dimension];
  }

  CursorWithoutBoundsCheck(const BoundingBoxType& b_box,
                           OffsetTableType o_table)
      : offset_table(o_table), ImagePolicyType() {
    bounding_box = b_box;
    size = ImagePolicyType::convertBoundingBoxToSize(bounding_box);
    start_offset = 0;
    end_offset = offset_table[dimension];
  }
  OffsetType getOffset() const { return current_offset; }

 protected:
  void toNextPoint() { current_offset++; }
  void toPreviousPoint() { current_offset--; }
  void toNextLine() {
    current_offset = current_offset + this->size[0];
    toLineBegin();
  }
  void toPreviousLine() {
    current_offset = current_offset - this->size[0];
    toLineEnd();
  }
  void toLineBegin() {
    current_offset = current_offset - (current_offset | this->size[0]);
  }
  void toLineEnd() {
    current_offset =
        current_offset + (this->size[0] - (current_offset | this->size[0]));
  }
  void toBegin() { current_offset = start_offset; }
  void toEnd() { current_offset = end_offset; }

  bool isEnd() { return current_offset >= end_offset; }

  OffsetType start_offset;
  OffsetType end_offset;
  OffsetType current_offset;
  OffsetTableType offset_table;
  SizeType size;
};

template <size_t dimension,
          class ValueType,
          class ImagePolicyType = ImagePolicy<dimension>,
          class CursorType = CursorWithoutBoundsCheck<dimension>>
class Iterator : public CursorType {
 public:
  using ImplicitCursorType = CursorType;
  using OffsetType = typename CursorType::OffsetType;
  using BoundingBoxType = typename CursorType::BoundingBoxType;
  using PointType = Point<dimension, ValueType>;
  using ImageType =
      image::Image<dimension, ValueType, ImagePolicyType, PointType>;

  Iterator(BoundingBoxType& bbox, ImageType& i_image)
      : CursorType(bbox), image(i_image) {}
  constexpr void moveBackward() { CursorType::toPreviousPoint(); }

  constexpr void moveForward() { CursorType::toNextPoint(); }

  constexpr void begin() { CursorType::toBegin(); }

  constexpr bool isEnd() { return CursorType::isEnd(); }

  void operator++() { CursorType::toNextPoint(); }

  void operator--() { CursorType::toPreviousPoint(); }

  constexpr void applyFunction(
      std::function<void(std::shared_ptr<PointType>)> function) {
    begin();
    while (!isEnd()) {
      function(getPoint());

      moveForward();
    }
  }

  constexpr OffsetType getOffset() { return CursorType::current_offset; }

  constexpr auto getPoint() {
    return image.template getValue<OffsetType, PointType>(getOffset());
  }

  constexpr auto getValue() { return image.getValue(getOffset()); }

 protected:
  ImageType image;
};

template <size_t dimension,
          class ValueType,
          class ImagePolicyType = ImagePolicy<dimension>,
          class CursorType = CursorWithoutBoundsCheck<dimension>,
          class NeighbourhoodCursorType = CursorWithBoundsCheck<dimension>,
          class InternalIteratorType = Iterator<dimension, ValueType>>
class NeighbourhoodIterator : public InternalIteratorType {
 public:
  using IndexType = typename InternalIteratorType::PointType::IndexType;
  using OffsetType = typename InternalIteratorType::PointType::OffsetType;

  using PointType = typename InternalIteratorType::PointType;
  using BoundingBoxType = typename InternalIteratorType::BoundingBoxType;
  using ImageType = typename InternalIteratorType::ImageType;

  constexpr void beginNeighbourhood() {
    n_cursor = std::make_unique<NeighbourhoodCursorType>(
        getLocalisedBoundingBox(),
        InternalIteratorType::image.getOffsetTable());
    n_cursor->toBegin();
  }
  constexpr bool isNeighbourhoodEnd() { return n_cursor->isEnd(); }
  constexpr void moveNeighbourhoodForward() { n_cursor->toNextPoint(); }
  constexpr void moveNeighbourhoodBackward() { n_cursor->toPreviousPoint(); }

  constexpr auto getNeighbourhoodOffset() { return n_cursor->current_offset; }

  NeighbourhoodIterator(BoundingBoxType& bbox,
                        ImageType& i_image,
                        size_t n_radius)
      : InternalIteratorType(bbox, i_image), neighbourhood_radius(n_radius) {}

  constexpr auto getNeighbourhoodPoint() {
    return InternalIteratorType::image.template getValue<OffsetType, PointType>(
        getNeighbourhoodOffset());
  }

  constexpr auto getNeighbourhoodIndex() {
    return ImagePolicyType::convertOffsetToIndex(CursorType::offset_table,
                                                 getNeighbourhoodOffset());
  }

  auto getNeighbourhoodValue() {
    return InternalIteratorType::image.getValue(getNeighbourhoodOffset());
  }

  void applyFunction(
      std::function<void(std::shared_ptr<PointType>,
                         std::vector<std::shared_ptr<PointType>>)> function) {
    this->begin();
    while (!this->isEnd()) {
      function(InternalIteratorType::getPoint(), getNeighbours());

      this->moveForward();
    }
  }

  auto getNeighbours() {
    beginNeighbourhood();
    std::vector<std::shared_ptr<PointType>> neighbours;
    n_cursor->toBegin();
    while (!isNeighbourhoodEnd()) {
      auto val =
          InternalIteratorType::image.template getValue<OffsetType, PointType>(
              getNeighbourhoodOffset());
      neighbours.push_back(val);
      moveNeighbourhoodForward();
    }
    return neighbours;
  }

 private:
  size_t neighbourhood_radius;
  std::unique_ptr<NeighbourhoodCursorType> n_cursor{};

  constexpr auto getLocalisedBoundingBox() {
    BoundingBoxType bb;
    bb.fill(0);
    auto index = ImagePolicyType::convertOffsetToIndex(
        CursorType::offset_table, CursorType::current_offset);

    for (size_t i = 0; i < dimension; i++) {
      bb[i] = int(index[i]) - int(neighbourhood_radius) < 0
                  ? 0
                  : index[i] - neighbourhood_radius;
      bb[i + dimension] = index[i] + neighbourhood_radius + 1 > this->size[i]
                              ? this->size[i]
                              : index[i] + neighbourhood_radius + 1;
    }

    return bb;
  }
};

template <class ImageType, size_t nThreads = 1, size_t neighbourhoodRadius = 0>
struct IteratorFactory {
  using BoundingBoxType = typename ImageType::BoundingBoxType;
  using Self = IteratorFactory<ImageType, nThreads, neighbourhoodRadius>;
  static const size_t dimension = ImageType::dimension;
  using PixelValueType = typename ImageType::PixelValueType;
  using IteratorType = Iterator<dimension, PixelValueType>;
  using NeighbourhoodIteratorType =
      NeighbourhoodIterator<dimension, PixelValueType>;
  std::vector<BoundingBoxType> boxes;

  std::vector<IteratorType> rIterators;
  std::vector<NeighbourhoodIteratorType> nIterators;

  auto divideBoundingBoxForThreads() {
    while (boxes.size() < nThreads) {
      auto first = boxes.back();
      divideBoundingBoxByTwo(first);
    }
  }

  IteratorFactory(ImageType* im) : image(im) {
    boxes.push_back(image->getBoundingBox());
    divideBoundingBoxForThreads();
  }

  constexpr auto divideBoundingBoxByTwo(BoundingBoxType v) {
    BoundingBoxType firstHalf;
    BoundingBoxType secondHalf;
    boundingSplitIterate<0>(firstHalf, secondHalf, v);
    boxes.insert(boxes.begin(), firstHalf);
    boxes.insert(boxes.begin(), secondHalf);
    boxes.pop_back();
  }

  template <size_t loop>
  constexpr auto boundingSplitIterate(BoundingBoxType& fHalf,
                                      BoundingBoxType& sHalf,
                                      BoundingBoxType& main) {
    if constexpr (loop != dimension) {
      fHalf[loop] = main[loop];
      fHalf[loop + dimension] = ((main[loop + dimension] + main[loop]) / 2) +
                                (main[loop + dimension] % 2);
      sHalf[loop] = (main[loop + dimension] + main[loop]) / 2;
      sHalf[loop + dimension] = main[loop + dimension];
      boundingSplitIterate<loop + 1>(fHalf, sHalf, main);
    }
  }

  constexpr void createIterators() {
    for (auto&& bb : boxes) {
      if constexpr (neighbourhoodRadius > 0) {
        nIterators.emplace_back(bb, image, neighbourhoodRadius);
      } else {
        rIterators.emplace_back(bb, image);
      }
    }
  }

  template <typename Type>
  constexpr void iteratorFunction(Type iter) {
    iter.begin();
    while (!iter.isEnd()) {
      iter.moveForward();
    }
  }

  constexpr auto run() {
    if constexpr (neighbourhoodRadius > 0) {
      for (auto&& iterator : nIterators) {
        std::thread t(&Self::iteratorFunction<NeighbourhoodIteratorType>, this,
                      iterator);
        t.join();
      }
    } else {
      for (auto&& iterator : rIterators) {
        std::thread t(&Self::iteratorFunction<IteratorType>, this, iterator);
        t.join();
      }
    }
  }

  ImageType* image;
};

}  // namespace common::iterator
