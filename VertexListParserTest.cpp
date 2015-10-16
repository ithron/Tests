#define BOOST_SPIRIT_USE_PHOENIX_V3

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix.hpp>
#include <boost/fusion/adapted/std_tuple.hpp>

#include "UniformTuple.hh"
#include "FixedSizedTypeConversion.hh"

#include <array>
#include <iostream>
#include <string>

using Utilities::UniformTuple;

typedef std::array<double, 3> Vertex;
typedef std::array<unsigned int, 3> Triangle;

typedef std::vector<Vertex> VertexSet;
typedef std::vector<Triangle> TriangleSet;

typedef std::tuple<VertexSet, TriangleSet> Mesh;

template <class T> constexpr decltype(auto) getVertices(T &&mesh) {
  using std::get;
  return get<0>(std::forward<T>(mesh));
}
template <class T> constexpr decltype(auto) getTriangles(T &&mesh) {
  using std::get;
  return get<1>(std::forward<T>(mesh));
}

namespace std {
template <class T>
std::ostream &operator<<(std::ostream &os, std::array<T, 3> const &v) {
  os << "(" << v[0] << ", " << v[1] << ", " << v[2] << ")";
  return os;
}
} // namespace std

// clang-format off
namespace boost { namespace spirit { namespace traits {
// clang-format on

template <class T, std::size_t N>
struct is_container<std::array<T, N>> : mpl::false_ {};

template <class T, std::size_t N>
struct transform_attribute<std::array<T, N>, UniformTuple<T &, N>, qi::domain> {
  typedef UniformTuple<T &, N> type;
  typedef std::array<T, N> Array;
  static type pre(Array &v) { return Utilities::convertFixed<type>(v); }
  static void post(Array &, type const &) {}
  static void fail(Array &) {}
};

// clang-format off
}}}
// clang-forman on

namespace qi = boost::spirit::qi;
namespace ascii = boost::spirit::ascii;
namespace phoenix = boost::phoenix;

template <class Iterator, class VertexType>
struct ListGrammar : qi::grammar<Iterator, std::vector<VertexType>(std::string),
                                 ascii::blank_type> {

  typedef typename VertexType::value_type ValueType;

  ListGrammar() : ListGrammar::base_type(list) {
    using qi::auto_;
    using qi::eol;
    using qi::lit;
    using namespace qi::labels;

    listStart = lit(_r1) >> "start" >> +eol;
    listEnd = lit(_r1) >> "end" >> +eol;
    realVertex %= auto_ >> auto_ >> auto_ >> +eol;
    vertex %= realVertex;
    list %= listStart(_r1) >> *vertex >> listEnd(_r1);
  }

  qi::rule<Iterator, void(std::string), ascii::blank_type> listStart;
  qi::rule<Iterator, void(std::string), ascii::blank_type> listEnd;
  qi::rule<Iterator, VertexType(), ascii::blank_type> vertex;
  qi::rule<Iterator,
           UniformTuple<ValueType &, std::tuple_size<VertexType>::value>(),
           ascii::blank_type> realVertex;
  qi::rule<Iterator, std::vector<VertexType>(std::string), ascii::blank_type>
    list;
};

template <class Iterator>
struct UnusedListGrammar
  : qi::grammar<Iterator, qi::locals<std::string>, ascii::blank_type> {

  UnusedListGrammar() : UnusedListGrammar::base_type(list) {
    using qi::alnum;
    using qi::auto_;
    using qi::char_;
    using qi::eol;
    using qi::lexeme;
    using qi::lit;
    using namespace qi::labels;

    any = *(char_ - eol);
    listStart %= lexeme[+alnum] >> "start" >> +eol;
    listEnd = lit(_r1) >> "end" >> +eol;
    unusedLine = (any - listEnd(_r1)) >> +eol;
    list = listStart[_a = _1] >> *unusedLine(_a) >> listEnd(_a);
  }

  qi::rule<Iterator, std::string(), ascii::blank_type> listStart;
  qi::rule<Iterator, void(std::string), ascii::blank_type> listEnd;
  qi::rule<Iterator, void(std::string), ascii::blank_type> unusedLine;
  qi::rule<Iterator, qi::locals<std::string>, ascii::blank_type> list;
  qi::rule<Iterator, ascii::blank_type> any;
};

template <class Iterator>
struct MeshGrammer : qi::grammar<Iterator, Mesh(), ascii::blank_type> {

  MeshGrammer() : MeshGrammer::base_type(start) {
    using namespace std::string_literals;
    using namespace qi::labels;

    vertexSet %= vertexList("vertices"s);
    triangleSet %= triangleList("triangles"s);
    skip_ = skipList;
    skip = skip_ - (vertexSet | triangleSet);

    start = (vertexSet[phoenix::bind(
               [](Mesh &msh, VertexSet const &v) { getVertices(msh) = v; },
               _val, _1)] >>
             *skip) ^
            (triangleSet[phoenix::bind([](Mesh &msh, TriangleSet const &t) {
              getTriangles(msh) = t;
            }, _val, _1)] >> *skip) ^ *skip;
  }

  ListGrammar<Iterator, Vertex> vertexList;
  ListGrammar<Iterator, Triangle> triangleList;
  UnusedListGrammar<Iterator> skipList;
  qi::rule<Iterator, Mesh(), ascii::blank_type> start;
  qi::rule<Iterator, ascii::blank_type> skip;
  qi::rule<Iterator, ascii::blank_type> skip_;
  qi::rule<Iterator, VertexSet(), ascii::blank_type> vertexSet;
  qi::rule<Iterator, TriangleSet(), ascii::blank_type> triangleSet;
};

int main(int, char **) {
  using namespace std::string_literals;

  auto input = "triangles start\n"
               "0 1 2\n"
               "0 2 3\n"
               "1 3 2\n"
               "0 3 1\n"
               "triangles end\n\n \n"
               "colors start\n"
               " 1 23  444\n\n"
               "colors end\n"
               "vertices start\n"
               "1 2 3\n"
               "3.1 2.2 3.3\n"
               "3e-2 1e3 -5e-5\n"
               "25.3 -2.1 5\n"
               "vertices end\n"s;
  typedef typename decltype(input)::iterator Iterator;

  MeshGrammer<Iterator> meshGrammar;
  Mesh mesh;

  Iterator iter = input.begin();
  bool const res =
    qi::phrase_parse(iter, input.end(), meshGrammar, ascii::blank, mesh);

  std::cout << "Matched: " << std::boolalpha << (res && iter == input.end())
            << std::endl;
  std::cout << "Mesh = (" << std::endl
            << "\tV = {" << std::endl;
  for (auto const &vi : getVertices(mesh)) {
    std::cout << "\t\t" << vi << "," << std::endl;
  }
  std::cout << "\t}," << std::endl
            << "\tT = {" << std::endl;
  for (auto const &ti : getTriangles(mesh)) {
    std::cout << "\t\t" << ti << "," << std::endl;
  }
  std::cout << "\t}" << std::endl
            << ")" << std::endl;

  return EXIT_SUCCESS;
}
