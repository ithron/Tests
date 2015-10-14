#ifndef SRE_UNIFORM_TUPLE_HH
#define SRE_UNIFORM_TUPLE_HH

#include <tuple>

namespace Utilities {

namespace detail_ {

template <class T, std::size_t N, class... Tail> struct UniformTupleHelper {
  typedef UniformTupleHelper<T, N - 1, T, Tail...> type;
};

template <class T, class... Tail> struct UniformTupleHelper<T, 0, Tail...> {
  typedef std::tuple<Tail...> type;
};

template <class... Args> struct FirstTypeInList {
  typedef typename std::tuple_element<0, std::tuple<Args...>>::type type;
};
} // namespace detail_

template <class T, std::size_t N>
using UniformTuple = typename detail_::UniformTupleHelper<T, N>::type;

template <class... Args>
constexpr UniformTuple<typename detail_::FirstTypeInList<Args...>::type,
                       sizeof...(Args)>
makeUniformTuple(Args &&... args) {
  return {{std::forward<Args>(args)...}};
}
} // namespace Utilities

#endif // SRE_UNIFORM_TUPLE_HH
