#include "assembly.hpp"

#include <boost/graph/adjacency_list.hpp>
// #include <boost/graph/depth_first_search.hpp>
#include <boost/graph/topological_sort.hpp>


namespace std {

// range adaptor
template<class Iterator>
static Iterator begin(const std::pair<Iterator, Iterator>& range) { return range.first; }


template<class Iterator>
static Iterator end(const std::pair<Iterator, Iterator>& range) { return range.second; }

}


namespace sofa {
namespace assembly {

struct vertex_type {
    using state_type = const core::BaseState*;
    state_type state;

    bool is_mechanical;
};


struct edge_type {
    using mapping_type = const core::BaseMapping*;
    mapping_type mapping;
    
    std::size_t index;
};

template<class Vertex, class Edge>
using graph = boost::adjacency_list<boost::vecS, boost::vecS,
                                    boost::bidirectionalS,
                                    Vertex, Edge>;


using graph_type = graph<vertex_type, edge_type>;



struct assembly_error : std::runtime_error {
    using std::runtime_error::runtime_error;
};


// TODO: inherit from Visitor directly?
struct Visitor : public simulation::MechanicalVisitor {

    graph_type& graph;
    std::map< vertex_type::state_type, std::size_t > table;

    Visitor(graph_type& graph)
        : simulation::MechanicalVisitor(nullptr),
        graph(graph) { }
    
    virtual Visitor::Result processNodeTopDown(simulation::Node* node) {
        if( auto* state = node->mechanicalState.get() ) {
            const auto err = table.emplace(state, num_vertices(graph));

            if(err.second) {
                const bool is_mechanical = node->mass.get() || node->forceField.size();
                
                const vertex_type prop = {state, is_mechanical};
                const std::size_t v = add_vertex(prop, graph);
                assert(v == err.first->second && "bad vertex index");
            }
        }

        return RESULT_CONTINUE;        
    }
    
	virtual void processNodeBottomUp(simulation::Node* node) {
        if(auto* mapping = node->mechanicalMapping.get()) {
            const auto* state = node->mechanicalState.get();
            if(!state) throw assembly_error("no output dof");
            
            const auto src = table.find(state);
            if(src == table.end()) throw assembly_error("unknown output dof");
            
            std::size_t index = 0;
            for(auto* state : mapping->getFrom()) {
                const auto dst = table.find(state);
                if(dst == table.end()) throw assembly_error("unknown input dof");
                
                const edge_type prop = {mapping, index};
                add_edge(dst->second, src->second, prop, graph);

                ++index;
            }
        }
    }
    
};




static graph_type create_graph(core::objectmodel::BaseContext* ctx) {

    // fill kinematic graph
    graph_type graph;
    
    Visitor visitor(graph);
    ctx->executeVisitor(&visitor);

    // topological sort
    std::vector<std::size_t> bottom_up;

    boost::vector_property_map<boost::default_color_type> cm(num_vertices(graph));    
    topological_sort(graph, std::back_inserter(bottom_up), boost::color_map(cm));

    // propagate is_mechanical bottom-up
    for(std::size_t v : bottom_up) {
        if(graph[v].is_mechanical) {
            
            for(std::size_t s : adjacent_vertices(v, graph)) {
                graph[s].is_mechanical = true;
            }
        }
    }


    // prune graph keeping only mechanical nodes
    std::vector<std::size_t> map;
    map.reserve(num_vertices(graph));
    
    for(std::size_t v : vertices(graph) ) {
        if(graph[v].is_mechanical) {
            map.emplace_back(map.size());
        } else {
            clear_vertex(v, graph);
        }
    }

    // compact graph
    graph_type res(map.size());
    
    for(std::size_t v : vertices(graph) ) {
        if(graph[v].is_mechanical) {
            const std::size_t s = map[v];

            res[s] = graph[v];

            for(auto e : out_edges(v, graph)) {
                add_edge(s, map[target(e, graph)], graph[e], res);
            }
        }            
    }

    return res;
}



system_type assemble_system(core::objectmodel::BaseContext* ctx,
                            const core::MechanicalParams* mparams) {

    graph_type graph = create_graph(ctx);
    
    // TODO obtain mapping chunks

    // TODO obtain mass/stiffness chunks

    // TODO obtain forces + geometric stiffness chunks

    return {};
}


}
}
