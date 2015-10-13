#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix.hpp>
#include <boost/fusion/adapted/std_tuple.hpp>

#include <array>
#include <iostream>
#include <string>

typedef std::array<double, 3> Vertex;

namespace boost { namespace spirit { namespace traits {
  template<> struct is_container<Vertex> : mpl::false_ {};

  template <>
  struct transform_attribute<Vertex, std::tuple<double &, double &, double &>,
                             qi::domain> {
    typedef std::tuple<double &, double &, double &> type;
    static type pre(Vertex &v) { return type(v[0], v[1], v[2]); }
    static void post(Vertex &, type const &) {}
    static void fail(Vertex &) {}
  };
}}}

namespace qi = boost::spirit::qi;
namespace ascii = boost::spirit::ascii;

template <class Iterator>
struct ListGrammar : qi::grammar<Iterator, std::vector<Vertex>(),
                                 qi::locals<std::string>, ascii::blank_type> {
  ListGrammar() : ListGrammar::base_type(list) {
    using qi::alnum;
    using qi::double_;
    using qi::eol;
    using qi::lexeme;
    using qi::lit;
    using qi::omit;
    using qi::repeat;
    using namespace qi::labels;

    listStart %= lexeme[+alnum] >> "start" >> eol;
    listEnd %= lit(_r1) >> "end" >> eol;
    realVertex %= double_ >> double_ >> double_ >> eol;
    vertex %= realVertex;
    list %= omit[listStart[_a = _1]] >> *vertex >> listEnd(_a);

    BOOST_SPIRIT_DEBUG_NODE(listStart);
    BOOST_SPIRIT_DEBUG_NODE(listEnd);
    BOOST_SPIRIT_DEBUG_NODE(list);
  }

  qi::rule<Iterator, std::string(), ascii::blank_type> listStart;
  qi::rule<Iterator, void(std::string), ascii::blank_type> listEnd;
  qi::rule<Iterator, Vertex(), ascii::blank_type> vertex;
  qi::rule<Iterator, std::tuple<double &, double &, double &>(),
           ascii::blank_type> realVertex;

  qi::rule<Iterator, std::vector<Vertex>(), qi::locals<std::string>,
           ascii::blank_type> list;
};

int main(int, char **) {
  using namespace std::string_literals;

  auto input =
    "vertices start\n"
    "1 2 3\n"
    "3.1 2.2 3.3\n"
    "3e-2 1e3 -5e-5\n"
    "vertices end\n"s;
  typedef typename decltype(input)::iterator Iterator;

  ListGrammar<Iterator> list;

  std::vector<Vertex> v;
  bool const res =
      qi::phrase_parse(input.begin(), input.end(), list, ascii::blank, v);

  std::cout << "Matched: " << std::boolalpha << res << std::endl;
  for (auto const &vi : v) {
    std::cout << "(" << vi[0] << ", " << vi[1] << ", " << vi[2] << ")"
              << std::endl;
  }

  return EXIT_SUCCESS;
}
