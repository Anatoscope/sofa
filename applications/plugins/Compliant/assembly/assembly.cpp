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

    using state_type = core::behavior::BaseMechanicalState*;
    state_type state = nullptr;

    // stage1
    bool is_mechanical = false;

    using mapping_type = core::BaseMapping*;
    mapping_type mapping = nullptr;
    
    // stage2
    std::size_t offset = 0, size = 0;
};


struct edge_type {
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

        // construct graph vertices
        if( auto* state = node->mechanicalState.get() ) {
            const auto err = table.emplace(state, num_vertices(graph));

            if(err.second) {
                vertex_type prop;
                
                prop.state = state;
                prop.is_mechanical = node->mass.get() || node->forceField.size();
                
                const std::size_t v = add_vertex(prop, graph); (void) v;
                assert(v == err.first->second && "bad vertex index");
            }
        }

        return RESULT_CONTINUE;        
    }
    
	virtual void processNodeBottomUp(simulation::Node* node) {

        // construct graph edges
        if(auto* mapping = node->mechanicalMapping.get()) {
            auto* state = node->mechanicalState.get();
            if(!state) throw assembly_error("no output dof");
            
            const auto dst = table.find(state);
            if(dst == table.end()) throw assembly_error("unknown output dof");

            // remember this dof is mapped
            graph[dst->second].mapping = mapping;
            
            std::size_t index = 0;
            for(auto* state : mapping->getFrom()) {
                auto* mstate = state->toBaseMechanicalState();
                if(!mstate) continue;
                
                const auto src = table.find(mstate);
                if(src == table.end()) throw assembly_error("unknown input dof");
                
                const edge_type prop = {index};

                // edges go from input to outputs, so that topological sort
                // treats inputs first
                add_edge(src->second, dst->second, prop, graph);
                
                ++index;
            }
        }
    }
    
};



static graph_type prune_graph(const graph_type& graph) {
    
    // associate mechanical vertices
    std::vector<std::size_t> map;
    map.reserve(num_vertices(graph));
    
    for(std::size_t v : vertices(graph) ) {
        if(graph[v].is_mechanical) {
            map.emplace_back(map.size());
        } else {
            // clear_vertex(v, graph);
        }
    }

    // connect compacted graph
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


static void topological_sort(std::vector<std::size_t>& ordering, const graph_type& graph) {
    ordering.clear();
    ordering.reserve(num_vertices(graph));
    
    boost::vector_property_map<boost::default_color_type> cm(num_vertices(graph));    
    topological_sort(graph, std::back_inserter(ordering), boost::color_map(cm));
}
                             



static graph_type create_graph(core::objectmodel::BaseContext* ctx) {

    // fill kinematic graph
    graph_type graph;
    
    Visitor visitor(graph);
    ctx->executeVisitor(&visitor);

    // topological sort
    std::vector<std::size_t> top_down;

    topological_sort(top_down, graph);
    
    // propagate is_mechanical bottom-up
    for(std::size_t v : top_down) {
        if(graph[v].is_mechanical) {
            
            for(std::size_t s : inv_adjacent_vertices(v, graph)) {
                graph[s].is_mechanical = true;
            }
        }
    }

    // prune non-mechanical nodes
    return prune_graph(graph);
}



using real = SReal;
using triplet = Eigen::Triplet<real>;
using triplets_type = std::vector<triplet>;


template<class OutputIterator>
static void fill_mapping_chunk(OutputIterator out, const defaulttype::BaseMatrix* chunk, 
                               std::size_t row_off, std::size_t col_off) {
    // TODO
}


template<class OutputIterator>
static void fill_mapping(OutputIterator out, const graph_type& graph) {
    for(std::size_t v : vertices(graph) ) {
        if(graph[v].mapping) {

            auto* chunks = graph[v].mapping->getJs();
            
            for(auto e : in_edges(v, graph) ) {
                const std::size_t s = source(e, graph);
                fill_mapping_chunk(out, (*chunks)[graph[e].index], graph[v].offset, graph[s].offset);
            }

        } else {
            
        }
    }
}

template<class OutputIterator>
static void fill_mass(OutputIterator out, const graph_type& graph, real factor = 1) {
    
}


template<class OutputIterator>
static void fill_stiffness(OutputIterator out, const graph_type& graph, real factor = 1) {

}


template<class OutputIterator>
static void fill_compliance(OutputIterator out, const graph_type& graph, real factor = 1) {
    
}


static std::size_t number_vertices(graph_type& graph, const std::vector<std::size_t>& top_down) {

    std::size_t off = 0;
    for(std::size_t v : top_down) {
        const std::size_t size = graph[v].state->getMatrixSize();

        graph[v].offset = off;
        graph[v].size = size;

        off += size;
    }

    return off;
}



system_type assemble_system(core::objectmodel::BaseContext* ctx,
                            const core::MechanicalParams* mparams) {

    graph_type graph = create_graph(ctx);

    // order graph
    std::vector<std::size_t> top_down;
    topological_sort(top_down, graph);

    // fill offset/size for vertices
    number_vertices(graph, top_down);
    
    // TODO obtain and concatenate mappings chunks
    triplets_type Js;
    fill_mapping(std::back_inserter(Js), graph);
    
    // TODO obtain mass/stiffness chunks
    triplets_type Hs;
    
    // TODO obtain forces + geometric stiffness chunks

    return {};
}


}
}
