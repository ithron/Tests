#ifndef SRE_FIXED_SIZED_TYPE_CONVERSION_HH
#define SRE_FIXED_SIZED_TYPE_CONVERSION_HH

#include <array>
#include <type_traits>
#include <utility>
#include <tuple>

namespace Utilities {

namespace detail_ {

using std::get;
template <std::size_t I, class T>
constexpr auto get_(T &&t, int) -> decltype(get<I>(std::forward<T>(t))) {
  return get<I>(std::forward<T>(t));
}

template <std::size_t I, class T>
constexpr auto get_(T &&t, long) -> decltype(std::forward<T>(t)[I]) {
  return std::forward<T>(t)[I];
}

template <std::size_t I, class T> constexpr decltype(auto) get_(T &&t) {
  return get_<I>(std::forward<T>(t), 0);
}

namespace test_ {
struct Test {
  constexpr Test(int a, int b) : a{{a, b}} {}
  constexpr int operator[](std::size_t i) const { return a[i]; }
  std::array<int, 2> a;
};
} // namespace test_

static_assert(get_<0>(std::array<int, 2>{{1, 2}}) == 1,
              "Implementation of get is wrong.");
static_assert(get_<0>(std::tuple<int, char>{1, 'a'}) == 1,
              "Implementation of get is wrong.");
static_assert(get_<0>(test_::Test(23, 42)) == 23,
              "Implementatin of get is wrong");

template <class To, class From, std::size_t... I>
constexpr To convertFixedHelper(From &&from, std::index_sequence<I...>) {
  return To(get_<I>(std::forward<From>(from))...);
}

} // namespace detail_

/// Converts fixed size type From to fixed size type To.
///
/// \tparam To target type. To{a1, a2, ..., aN} with muste be a valid
/// expression
/// \tparam From source type. either get<idx>(f) or f[idx] with f object of
/// type From must a valid expression
/// \tparam N size of source (and target) type. Can be obmitted if an
/// specialization of std::tuple_size exist for From.
/// \param from object to convert.
/// \return object of type To initialized with the elements from[0], ...
/// from[N].
template <
  class To,
  class From,
  std::size_t N = std::tuple_size<typename std::decay<From>::type>::value>
constexpr To convertFixed(From &&from) {
  return detail_::convertFixedHelper<To>(std::forward<From>(from),
                                         std::make_index_sequence<N>{});
}

///////////////////////////////////////////////////////////////////////////////
// Static tests

static_assert(convertFixed<std::tuple<int, int>>(std::array<int, 2>{
                {23, 42}}) == std::make_tuple(int(23), int(42)),
              "oonvertFixed broken");

static_assert(convertFixed<std::tuple<int, int>, detail_::test_::Test, 2>(
                detail_::test_::Test(23, 42)) ==
                std::make_tuple(int(23), int(42)),
              "convertFixed broken");

} // namespace Utilities

#endif // SRE_FIXED_SIZED_TYPE_CONVERSION_HH
