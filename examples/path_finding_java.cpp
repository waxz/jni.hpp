//
// Created by waxz on 23-3-31.
//

#include "string_util.h"
#include "nlohmann/json.hpp"

#include "jni_utils.h"



#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/named_function_params.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/array.hpp>
#include <array>
#include <utility>
#include <algorithm>
#include <iostream>


#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths_no_color_map.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>


/*
use graphviz

 sudo apt-get install graphviz
 dot -Tpng test.dot -o test.png
 dot -Tsvg test.dot -o test.svg
 dot test.dot -Tpdf -o test.pdf
 */

struct Vertex {
    float x;
    float y;
    int id;
    std::string name;
};
struct EdgePair{
    size_t id_start;
    size_t id_end ;
    float dist;
};

struct PathFinding{
    typedef boost::property<boost::edge_weight_t, float> Edge;
    using graph_t = boost::adjacency_list<boost::listS, boost::vecS, boost::directedS, Vertex, Edge>;
    using vertex_t = boost::graph_traits<graph_t>::vertex_descriptor;
    using edge_t = boost::graph_traits<graph_t>::edge_descriptor;

    graph_t g; // construct a graph object

    std::vector<vertex_t> vertex_vec;
    std::vector<float> distances;
    std::vector<vertex_t> pMap;
    std::vector<vertex_t> path;
    std::vector<vertex_t> forward_path;
    std::vector<size_t> forward_path_id;

    bool write_dot = false;


    void clear(){
        g.clear();
        vertex_vec.clear();
        distances.clear();
        pMap.clear();
        path.clear();
        forward_path.clear();
        forward_path_id.clear();
    }

    int createGraph(const std::vector<Vertex>& node_list, std::vector<EdgePair>& edge_list){
        g.clear();

        size_t node_num = node_list.size();
        size_t edge_num = edge_list.size();

        vertex_vec.resize(node_num);
        for(size_t i = 0; i < node_num ;i++){
            if(node_list[i].id == i){
                vertex_vec[i] =  boost::add_vertex(node_list[i], g);
            }else{
                return -1;
            }
        }

        for(size_t i = 0; i < edge_num ;i++){
            if(edge_list[i].id_start < node_num
               && edge_list[i].id_end < node_num
//               && edge_list[i].dist > 0.0 && edge_list[i].dist < 100.0
                    ){
                if(edge_list[i].id_end != edge_list[i].id_start){
                    boost::add_edge(vertex_vec[edge_list[i].id_start], vertex_vec[edge_list[i].id_end], {edge_list[i].dist}, g);
                    std::cout << "add edge: " << edge_list[i].id_start << " to " << edge_list[i].id_end << ", distance " << edge_list[i].dist << "\n";
                }
            }else{
                return -2;
            }
        }



        std::cout << "write_graphviz:\n";
        boost::write_graphviz(std::cout, g, [&](auto &out, auto v) {
                                  out << "[label=\"" << g[v].name << "\"]";
                              },
                              [&](auto &out, auto e) {
                                  out << "[distance=\"" <<
                                      boost::get(boost::edge_weight,g,e) << "\"]";
                              });

        if(write_dot){
            std::ofstream dotfile;
            dotfile.open("a_test_bgl.dot");
            boost::write_graphviz(dotfile, g, [&](auto &out, auto v) {

                                      char buffer[200];
                                      sprintf(buffer,R"([shape=box, style=filled, fillcolor=yellow, label="%s", pose="%.3f,%.3f"])",g[v].name.c_str(),g[v].x, g[v].y );

                                      out << buffer;
                                  },
                                  [&](auto &out, auto e) {
                                      out << "[label=\"" <<
                                          boost::get(boost::edge_weight,g,e) << "\"]";
                                  });
        }

        return 0;
    }

    int findPath(size_t id_start,size_t id_end){
        const size_t numVertices = boost::num_vertices(g);
        size_t node_num = vertex_vec.size();

        if(numVertices < 2){
            return -1;
        }
        if(id_start >= node_num
           &&id_end >= node_num
           && id_start == id_end
                ){
            return -2;
        }

        {


            distances.resize(numVertices);
            pMap.resize(numVertices);

            auto distanceMap = predecessor_map(
                    make_iterator_property_map(pMap.begin(), boost::get(boost::vertex_index, g))).distance_map(
                    make_iterator_property_map(distances.begin(), boost::get(boost::vertex_index, g)));

            auto source = vertex_vec[id_start];
            auto destination =  vertex_vec[id_end];

            boost::dijkstra_shortest_paths(g, source, distanceMap);

            path.clear();
            forward_path.clear();
            forward_path_id.clear();
            vertex_t current = destination;

            if( (pMap[current] != current)){
                while (  (pMap[current] != current) && (current != source) ) {
                    path.push_back(current);
                    current = pMap[current];
                }
            }

            if(current == source ){
                std::cout << "success to find path from " << g[destination].name << " to " << g[source].name   << std::endl;
                path.push_back(source);
            }else{
                std::cout << "fail to find path from " << g[destination].name << " to " << g[source].name   << std::endl;
                return -3;
            }
            std::copy(path.rbegin(), path.rend(), std::back_inserter(forward_path));
            forward_path_id.resize(forward_path.size());


            std::cout << "forward_path : \n";
            for (auto &p1: forward_path) {

                std::cout << g[p1].id << ", " << g[p1].name << ", distance: " << distances[p1] << "\n";
            }
            std::cout << "end of forward_path\n";

            std::transform(forward_path.begin(),forward_path.end(), forward_path_id.begin(), [this](auto& v){
                return g[v].id;
            });

            if(write_dot){
                std::ofstream dotfile;
                dotfile.open("a_test_bgl_path.dot");
                boost::write_graphviz(dotfile, g, [&](auto &out, auto v) {

                                          char buffer[200];
                                          auto it = std::find(forward_path.begin(),forward_path.end(), v);
                                          if(it != forward_path.end()){
                                              if(v == source){
                                                  sprintf(buffer,R"([shape=box, style=filled, fillcolor=red, color=green, label="source: %s", pos="%.3f,%.3f"])",g[v].name.c_str(),g[v].x, g[v].y );

                                              }else if(v == destination){
                                                  sprintf(buffer,R"([shape=box, style=filled, fillcolor=red, color=green, label="destination: %s", pos="%.3f,%.3f"])",g[v].name.c_str(),g[v].x, g[v].y );

                                              }else{
                                                  sprintf(buffer,R"([shape=box, style=filled, fillcolor=green, color=green, label="%s", pos="%.3f,%.3f"])",g[v].name.c_str(),g[v].x, g[v].y );

                                              }


                                          } else{
                                              sprintf(buffer,R"([shape=box, style=filled, fillcolor=yellow, label="%s", pose="%.3f,%.3f"])",g[v].name.c_str(),g[v].x, g[v].y );
                                          }

                                          out << buffer;
                                      },
                                      [&](auto &out, auto e) {
                                          out << "[label=\"" <<
                                              boost::get(boost::edge_weight,g,e) << "\"]";
                                      });
            }



        }
        return 0;
    }


};


inline void to_json(nlohmann::json& j, const Vertex& p)
{
    j = {{"x",p.x},
         {"y",p.y},
         {"id",p.id},
         {"name",p.name}
    };
}


inline void from_json(const nlohmann::json& j, Vertex& p) {
    j.at("x").get_to(p.x);
    j.at("y").get_to(p.y);
    j.at("name").get_to(p.name);
    j.at("id").get_to(p.id);

}

inline void to_json(nlohmann::json& j, const EdgePair& p)
{
    j = {{"id_start",p.id_start},
         {"id_end",p.id_end},
         {"dist",p.dist}
    };
}


inline void from_json(const nlohmann::json& j, EdgePair& p) {
    j.at("id_start").get_to(p.id_start);
    j.at("id_end").get_to(p.id_end);
    j.at("dist").get_to(p.dist);
}



void test_boost(){
    enum { topLeft, topRight, bottomRight, bottomLeft };

    std::array<std::pair<int, int>, 4> edges{{
                                                     std::make_pair(topLeft, topRight),
                                                     std::make_pair(topRight, bottomRight),
                                                     std::make_pair(bottomRight, bottomLeft),
                                                     std::make_pair(bottomLeft, topLeft)
                                             }};

    typedef boost::adjacency_list<boost::setS, boost::vecS,
            boost::undirectedS> graph;
    graph g{edges.begin(), edges.end(), 4};

    boost::array<int, 4> predecessors;
    predecessors[bottomRight] = bottomRight;

    boost::breadth_first_search(g, bottomRight,
                                boost::visitor(
                                        boost::make_bfs_visitor(
                                                boost::record_predecessors(predecessors.begin(),
                                                                           boost::on_tree_edge{}))));

    int p = topLeft;
    while (p != bottomRight)
    {
        std::cout << p << '\n';
        p = predecessors[p];
    }
    std::cout << p << '\n';

}



struct PathFinder: public JNIBase{
    static constexpr auto Name() { return "NextRobot/PathFinder"; }
//    static TestEnv test_env;

    PathFinding finder;

    PathFinder(JNIEnv& env):JNIBase(env) {
        std::cout << "Native peer initialized" << std::endl;
    }
    ~PathFinder(){

    }
    void close(JNIEnv&){

        finder.clear();
    }
    jni::jboolean  ok(jni::JNIEnv& env) {
        return true;
    }

    /*

     */
    jni::jint createGraph(jni::JNIEnv& env, jni::String& node_list_json_str, jni::String& edge_list_json_str){
        std::u16string  node_uStr16 =  std::get<0>(jni::GetStringChars(env, *node_list_json_str)).get();
        std::string node_str= common::toUTF8(node_uStr16);
        std::u16string  edge_uStr16 =  std::get<0>(jni::GetStringChars(env, *edge_list_json_str)).get();
        std::string edge_str= common::toUTF8(edge_uStr16);


        std::vector<Vertex> node_list = nlohmann::json::parse(node_str);
        std::vector<EdgePair> edge_list = nlohmann::json::parse(edge_str);

        return finder.createGraph(node_list, edge_list);
    }

    jni::Local<jni::String>  solve(jni::JNIEnv& env, jni::jint id_start,jni::jint id_end){


        size_t c_id_start = id_start;

        size_t c_id_end = id_end;

        int rt = finder.findPath(c_id_start,c_id_end);
        std::string  std_str;


        if(rt == 0 ){
            auto& path = finder.forward_path_id;
            nlohmann::json path_json = path;
            std_str = path_json.dump();

            std::cout << "success std_str: " << std_str << std::endl;


        }else{
            std::cout << "fail std_str: " << std_str << std::endl;

        }
        jni::Local<jni::String> std_string_to_java_string = jni::Make<jni::String>( env, std_str );
        return std_string_to_java_string;
    }
    jni::Local<jni::String>  hello(jni::JNIEnv& env){

        return jni::Make<jni::String>( env, "Hello from JNI !" );

        /*
         import java.util.ArrayList;
import java.util.List;
import java.util.stream.IntStream;
         */
        struct List { static constexpr auto Name() { return "java/util/List"; } };
        struct ArrayList { static constexpr auto Name() { return "java/util/ArrayList"; } };
        struct IntStream { static constexpr auto Name() { return "java/util/stream/IntStream"; } };
        struct String      { static constexpr auto Name() { return "java/lang/String"; } };

        static auto& JList = jni::Class<List>::Singleton(env);
        static auto& JArrayList = jni::Class<ArrayList>::Singleton(env);
        static auto& JIntStream = jni::Class<IntStream>::Singleton(env);
        static auto& JString = jni::Class<String>::Singleton(env);

        const char* c_str = "Hello World";
        std::string std_str( c_str );
        jni::Local<jni::String> c_string_to_java_string = jni::Make<jni::String>( env, c_str );
        jni::Local<jni::String> std_string_to_java_string = jni::Make<jni::String>( env, std_str );
        std::string std_str_from_java_string = jni::Make<std::string>( env, c_string_to_java_string );
        return std_string_to_java_string;
        return jni::Make<jni::String>( env, "Hello from JNI !" );

    }
};



static void RegisterPeer(JavaVM* vm){
    jni::JNIEnv& env { jni::GetEnv(*vm) };
#define METHOD(MethodPtr, name) jni::MakeNativePeerMethod<decltype(MethodPtr), (MethodPtr)>(name)
    jni::RegisterNativePeer<PathFinder>(env, jni::Class<PathFinder>::Find(env), "peer",
                                     jni::MakePeer<PathFinder>,
                                     "initialize",
                                        METHOD(&PathFinder::close, "close"),
                                        METHOD(&PathFinder::ok, "ok"),
                                     METHOD(&PathFinder::createGraph, "createGraph")
                                     , METHOD(&PathFinder::solve, "solve")
                                     ,METHOD(&PathFinder::hello, "hello")



    );



}



extern "C" JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void*)
{
    // peer
    RegisterPeer(vm);




    return jni::Unwrap(jni::jni_version_1_2);
}