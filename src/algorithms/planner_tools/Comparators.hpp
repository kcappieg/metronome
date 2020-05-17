#pragma once

/**
 *  Generic comparator functions
 */

#include <cstdint>
#include "Nodes.hpp"

namespace metronome {

/**
 *  Requires that Node type have function f and member g
 */
template <typename Node>
int fComparator(const Node& lhs, const Node& rhs) {
  if (lhs.f() < rhs.f()) return -1;
  if (lhs.f() > rhs.f()) return 1;
  if (lhs.g > rhs.g) return -1;
  if (lhs.g < rhs.g) return 1;
  return 0;
}

template <typename Domain>
int standardFComparator = fComparator<SearchNode<Domain>>;

/**
 *  Requires that Node type have member h
 */
template <typename Domain>
int standardHComparator(const SearchNode<Domain>& lhs, const SearchNode<Domain>& rhs) {
  if (lhs.h < rhs.h) return -1;
  if (lhs.h > rhs.h) return 1;
  return 0;
}

template<typename Node>
int gComparator(const Node& lhs, const Node& rhs) {
  if (lhs.g > rhs.g) return -1;
  if (lhs.g < rhs.g) return 1;
  return 0;
}

/**
 * Max comparator on g-value. Just reverses min comparator
 * @tparam Node
 * @param lhs
 * @param rhs
 * @return
 */
template<typename Node>
int gMaxComparator(const Node& lhs, const Node& rhs) {
  return -1 * gComparator<Node>(lhs, rhs);
}

} //namespace metronome
