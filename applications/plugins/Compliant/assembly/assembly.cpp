#include "assembly.hpp"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>

namespace sofa {
namespace component {

struct vertex_type {
    using state_type = const core::BaseState*;
    state_type state;

    vertex_type(state_type state = nullptr) : state(state) { }
    
    bool is_mechanical = false;
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
                const vertex_type prop(state);
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



void assemble(linearsolver::AssembledSystem& res,
              core::objectmodel::BaseContext* ctx,
              const core::MechanicalParams *mparams) {

    
    

}


}
}
