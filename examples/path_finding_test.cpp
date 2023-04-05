// Adapted from http://stackoverflow.com/questions/18563877/struggling-to-implement-simple-boostgraph-traversal

#include "jni_utils.h"



#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths_no_color_map.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

class MyVisitor : public boost::default_dfs_visitor {


private:
    boost::shared_ptr<std::vector<int> > vv;
};


void test_3(){
        struct Vertex { int value;};
        struct Edge { int value;};

        using namespace boost;
        using graph_t  = adjacency_list<listS, vecS, directedS, Vertex, Edge >;
//    using graph_t  = adjacency_list<listS, vecS, directedS, Vertex, property< edge_weight_t, Edge > >;

        using vertex_t = graph_traits<graph_t>::vertex_descriptor;
        using edge_t   = graph_traits<graph_t>::edge_descriptor;

    using Depths = std::map<vertex_t, size_t>;

    struct Recorder : public boost::base_visitor<Recorder> {
        using event_filter = boost::on_tree_edge;

        Depths& _ref;
        Recorder(Depths& r):_ref(r){}

        void operator()(Edge e, const graph_t& g) const {

        }
    };

    struct dfs_visitor : boost::default_dfs_visitor {
        using event_filter = boost::on_tree_edge;

        std::vector<vertex_t> vv;
        void initialize_vertex(vertex_t   s, graph_t const& g) const { std::cout << "Initialize: " << g[s].value << std::endl; }
        void start_vertex(vertex_t        s, graph_t const& g) const { std::cout << "Start:      " << g[s].value << std::endl; }
        void discover_vertex(vertex_t     s, graph_t const& g) const {

            std::cout << "Discover:   " << g[s].value << std::endl;
        }
        void finish_vertex(vertex_t       s, graph_t const& g) const { std::cout << "Finished:   " << g[s].value << std::endl; }
    };

        //Instantiate a graph
        graph_t g;

        // Create two vertices in that graph
        vertex_t u = boost::add_vertex(Vertex{123}, g);
        vertex_t v = boost::add_vertex(Vertex{456}, g);

        // Create an edge conecting those two vertices
        boost::add_edge(u, v, Edge{3}, g);


    /* INDEX MAP CREATION*/
    std::map<vertex_t, size_t> i_map;
    for (const auto& v : boost::make_iterator_range(vertices(g))) {
        i_map.emplace(v, i_map.size());
    }

    auto ipmap = boost::make_assoc_property_map(i_map);

    /* COLOR MAP CREATION */
    std::vector<boost::default_color_type> c_map(num_vertices(g));
    auto cpmap = boost::make_iterator_property_map(c_map.begin(), ipmap);

    dfs_visitor vis{};

    Depths depths;
    Recorder r{depths};
    std::cout << "depth_first_search 1" << std::endl;
    boost::depth_first_search(g,
                              boost::visitor(vis).vertex_index_map(ipmap).color_map(cpmap)
                              );



    std::cout << "depth_first_search 2" << std::endl;


    boost::write_graphviz(std::cout, g, [&] (auto& out, auto v) {
                                  out << "[label=\"" << g[v].value << "\"]";
                              },
                              [&] (auto& out, auto e) {
                                  out << "[label=\"" << g[e].value<< "\"]";
                              });

    std::ofstream dotfile;
    dotfile.open("a_test_bgl.dot");

    boost::write_graphviz(dotfile, g, [&] (auto& out, auto v) {
                              out << "[label=\"" << g[v].value << "\"]";
                          },
                          [&] (auto& out, auto e) {
                              out << "[label=\"" << g[e].value<< "\"]";
                          });


        std::cout << std::flush;

    std::vector< int > d(boost::num_vertices(g));
    std::vector< vertex_t > p(boost::num_vertices(g));

//    boost::dijkstra_shortest_paths(g, u,
//                            predecessor_map(boost::make_iterator_property_map(
//                                    p.begin(), get(boost::vertex_index, g)))
//                                    .distance_map(boost::make_iterator_property_map(
//                                            d.begin(), get(boost::vertex_index, g))));
//


}

struct Node
{
    Node(const int t = 0) : m_t{t} {}
    int m_t;
};

/// A phylogeny is a graph of Node of different generations
/// connected by ancestry
using phylogeny = boost::adjacency_list<
        boost::vecS,
        boost::vecS,
        boost::undirectedS,
        Node
>;

using phylogeny_vd = boost::graph_traits<phylogeny>::vertex_descriptor;

class my_visitor : public boost::default_dfs_visitor
{
public:
    void discover_vertex(phylogeny_vd vd, const phylogeny& g) const
    {
        std::cout << g[vd].m_t << std::endl;
    }
};

//Create a phylogeny
//
//        b---c---d
//        |
//    a---+
//        |
//        e---f---g
//
// ---+---+---+---+---
//    0   1   2   3  t (generation)
phylogeny create_phylogeny()
{
    phylogeny p;
    const auto a = boost::add_vertex(Node(0), p);
    const auto b = boost::add_vertex(Node(1), p);
    const auto c = boost::add_vertex(Node(2), p);
    const auto d = boost::add_vertex(Node(3), p);
    const auto e = boost::add_vertex(Node(1), p);
    const auto f = boost::add_vertex(Node(2), p);
    const auto g = boost::add_vertex(Node(3), p);
    boost::add_edge(a, b, p);
    boost::add_edge(b, c, p);
    boost::add_edge(c, d, p);
    boost::add_edge(a, e, p);
    boost::add_edge(f, g, p);
    return p;
}

void test_1(){
    const phylogeny p = create_phylogeny();
    my_visitor v;
    boost::depth_first_search(
            p,
            boost::visitor(v)
    );

}

/// Do a depth-first search on a phylogeny,
/// print indices of vertices explored
int main()
{
    test_3();

}

