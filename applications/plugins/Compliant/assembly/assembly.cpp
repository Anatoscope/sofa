#include "assembly.hpp"

#include <boost/graph/adjacency_list.hpp>
// #include <boost/graph/depth_first_search.hpp>
#include <boost/graph/topological_sort.hpp>

#include <SofaEigen2Solver/EigenBaseSparseMatrix.h>


namespace std {

// range adaptors
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



struct edge_type { };

template<class Vertex, class Edge>
using graph = boost::adjacency_list<boost::vecS, boost::vecS,
                                    boost::bidirectionalS,
                                    Vertex, Edge>;


using graph_type = graph<vertex_type, edge_type>;



struct assembly_error : std::runtime_error {
    using std::runtime_error::runtime_error;
};

struct unimplemented : std::logic_error {
    unimplemented() : std::logic_error("unimplemented lol") { }
};



// TODO: inherit from Visitor directly?
struct Visitor : public simulation::MechanicalVisitor {

    graph_type& graph;
    std::map< vertex_type::state_type, std::size_t > table;

    Visitor(graph_type& graph, const core::MechanicalParams* mp)
        : simulation::MechanicalVisitor(mp),
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
            
            const auto src = table.find(state);
            if(src == table.end()) throw assembly_error("unknown output dof");

            // remember this dof is mapped
            graph[src->second].mapping = mapping;
            
            for(auto* state : mapping->getFrom()) {
                auto* mstate = state->toBaseMechanicalState();
                if(!mstate) continue;
                
                const auto dst = table.find(mstate);
                if(dst == table.end()) throw assembly_error("unknown input dof");
                
                // boost::graph topological sort orders nodes for a *dependency*
                // graph: edges go from outputs to inputs
                add_edge(src->second, dst->second, graph);
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


static void topological_sort(std::vector<std::size_t>& ordering, 
                             const graph_type& graph) {
    ordering.clear();
    ordering.reserve(num_vertices(graph));
    
    boost::vector_property_map<boost::default_color_type> cm(num_vertices(graph));    
    topological_sort(graph, std::back_inserter(ordering), boost::color_map(cm));
}
                             


// post-condition: all vertices have non-null state
static graph_type create_graph(core::objectmodel::BaseContext* ctx, 
                               const core::MechanicalParams* mp) {

    // fill kinematic graph
    graph_type graph;
    
    Visitor visitor(graph, mp);
    ctx->executeVisitor(&visitor);

    // topological sort
    std::vector<std::size_t> top_down;

    topological_sort(top_down, graph);
    
    // propagate is_mechanical bottom-up
    for(std::size_t v : top_down) {
        if(graph[v].is_mechanical) {
            
            for(std::size_t s : adjacent_vertices(v, graph)) {
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

using rmat = Eigen::SparseMatrix<real, Eigen::RowMajor>;
using cmat = Eigen::SparseMatrix<real, Eigen::ColMajor>;
    


template<class F> class jump_table;

template<class Ret, class Base, class ... Args>
class jump_table< Ret (*)(Base*, Args...) > {

    using func_type = Ret (*)(Base*, Args...);
    using table_type = std::map< std::type_index, func_type >;
    
    mutable table_type table;

    using check_type = bool (*)(Base*);
    std::vector< std::pair<check_type, func_type> > check;

public:
    
    template<class T>
    jump_table& add(func_type func) {
        // register type-erased check function
        check.emplace_back([](const Base* base) -> bool { return dynamic_cast<const T*>(base); }, 
                           func);
        return *this;
    }

    func_type get(Base* base) const {

        // lookup cached result
        auto it = table.find( typeid(*base) );
        if(it != table.end() ) return it->second;

        // slow checks + cache
        for(const auto& c : check) {
            if(c.first(base)) {
                table[ typeid(*base) ] = c.second;
                return c.second;
            }
        }
        
        throw std::runtime_error("no function registered");
    };
    
};


template<class Real, class OutputIterator>
static void fill_eigenbase_sparse_matrix_chunk(const defaulttype::BaseMatrix* self, OutputIterator out,
                                               std::size_t row_off, std::size_t col_off, real factor) {
    using cast_type = component::linearsolver::EigenBaseSparseMatrix<Real>;
    const auto& matrix = static_cast< const cast_type* >(self)->compressedMatrix;
    
    using matrix_type = typename std::decay<decltype(matrix)>::type;
    
    for(int i = 0, n = matrix.outerSize(); i < n; ++i) {
        for(typename matrix_type::InnerIterator it(matrix, i); it; ++it) {
            *out++ = {row_off + it.row(), col_off + it.col(), factor * it.value()};
        }
    }
    
};


template<class OutputIterator>
static void fill_base_matrix_chunk(const defaulttype::BaseMatrix* self, OutputIterator out,
                                   std::size_t row_off, std::size_t col_off, real factor) {
    throw unimplemented();
};


template<class OutputIterator>
static void fill_matrix_chunk(OutputIterator out, const defaulttype::BaseMatrix* chunk, 
                              std::size_t row_off, std::size_t col_off, real factor = 1) {
    
    using func_type = void (*) (const defaulttype::BaseMatrix*, OutputIterator, std::size_t, std::size_t, real);
    
    static const jump_table<func_type> jump = jump_table<func_type>()
        
        .template add<component::linearsolver::EigenBaseSparseMatrix<double> >
        ( fill_eigenbase_sparse_matrix_chunk<double, OutputIterator>)

        .template add<component::linearsolver::EigenBaseSparseMatrix<float> >
        ( fill_eigenbase_sparse_matrix_chunk<float, OutputIterator>)

        .template add<defaulttype::BaseMatrix>( fill_base_matrix_chunk<OutputIterator>);


    // dipatch
    jump.get(chunk)(chunk, out, row_off, col_off, factor);
}


template<class OutputIterator>
static void fill_identity_chunk(OutputIterator out, std::size_t row_off, std::size_t col_off, std::size_t size) {
    for(std::size_t i = 0; i < size; ++i) {
        *out++ = {row_off + i, col_off + i, 1};
    }
}


// TODO exploit projectors/masks
template<class OutputIterator>
static void fill_mapping(OutputIterator out, const graph_type& graph) {
    for(std::size_t v : vertices(graph) ) {
        if(graph[v].mapping) {

            auto* chunks = graph[v].mapping->getJs();

            std::size_t index = 0;
            for(auto e : out_edges(v, graph) ) {
                const std::size_t t = target(e, graph);
                fill_matrix_chunk(out, (*chunks)[index], graph[v].offset, graph[t].offset);
                ++index;
            }
            
        } else if(graph[v].state) {
            fill_identity_chunk(out, graph[v].offset, graph[v].offset, graph[v].size);
        }
    }
}


template<class OutputIterator>
struct base_matrix_adaptor : defaulttype::BaseMatrix {

    OutputIterator out;
    
    base_matrix_adaptor(OutputIterator out) : out(out) { }

    // lol
    Index rowSize() const { throw unimplemented(); }
    Index colSize() const { throw unimplemented(); }
    SReal element(Index, Index) const { throw unimplemented(); }
    void set(Index, Index, SReal) { throw unimplemented(); }    
    void resize(Index, Index) { throw unimplemented(); }
    void clear() { throw unimplemented(); }                

    void add(Index row, Index col, double value) {
        *out++ = {row, col, value};
    };
    
};



template<class OutputIterator>
struct multi_matrix_adaptor : core::behavior::MultiMatrixAccessor {
    // that's gotta be one of the crappiest APIs in the entire codebase, it's
    // really fascinating
    multi_matrix_adaptor(OutputIterator out, std::size_t offset)
        : bm(out),
          offset(offset) {

    }
    
    mutable base_matrix_adaptor<OutputIterator> bm;
    const std::size_t offset;
    
    int getGlobalDimension() const { throw unimplemented(); }
    int getGlobalOffset(const core::behavior::BaseMechanicalState*) const { 
        throw unimplemented();
    }
    
    InteractionMatrixRef getMatrix(const core::behavior::BaseMechanicalState* ,
                                   const core::behavior::BaseMechanicalState* ) const {
        throw unimplemented();
    }

    MatrixRef getMatrix(const core::behavior::BaseMechanicalState* ) const {
        // you gotta be f*cking kidding me, there's not even a suitable
        // constructor for using this thing
        MatrixRef res;
        res.matrix = &bm;
        res.offset = offset;
        return res;        
    }
    
};


static simulation::Node* node_cast(core::objectmodel::BaseContext* ctx) {
    assert(dynamic_cast<simulation::Node*>(ctx));        
    return static_cast<simulation::Node*>(ctx); 
}

using forcefield_type = core::behavior::BaseForceField*;

static forcefield_type node_compliance(simulation::Node* node) {
    for(auto* ff : node->forceField) {
        
        if(ff->isCompliance.getValue()) {
            if(node->forceField.size() > 1) {
                throw assembly_error("compliance must be the only forcefield in its node");
            }
            
            return ff;
        }
    }
    
    return nullptr;
}



template<class OutputIterator>
static void fill_forcefield(OutputIterator out, const graph_type& graph, const core::MechanicalParams* mp) {
    for(std::size_t v : vertices(graph) ) {
        if(!graph[v].state) continue;
        
        auto* node = node_cast(graph[v].state->getContext());
        
        for(auto* ff : node->forceField) {
            const multi_matrix_adaptor<OutputIterator> matrix(out, graph[v].offset);
            ff->addMBKToMatrix(mp, &matrix);
        }
    }
}


template<class OutputIterator>
static void fill_compliance(OutputIterator out, const graph_type& graph, const core::MechanicalParams* mp) {
    const real factor = -1 / mp->kFactor();
    
    for(std::size_t v : vertices(graph) ) {
        
        if(!graph[v].state) {
            // pairing node: fill pairing matrix
            auto it = adjacent_vertices(v, graph).first;

            assert(out_degree(v) == 2);            
            const std::size_t p1 = *it++, p2 = *it++;
            assert( graph[p1].size == graph[p2].size );
            assert( graph[p1].state == graph[p2].state );            

            const std::size_t size = graph[p1].size;

            fill_identity_chunk(out, graph[p1].offset, graph[p2].offset, size);
            fill_identity_chunk(out, graph[p2].offset, graph[p1].offset, size);            

        } else if(!out_degree(v, graph) ) {
            // independent node
            auto* node = node_cast(graph[v].state->getContext());

            if(auto* ff = node_compliance(node) ) {
                fill_matrix_chunk(out, ff->getComplianceMatrix(mp), 
                                  graph[v].offset, graph[v].offset, factor);
            }
        }
        
    }
}






static std::size_t number_vertices(graph_type& graph, const std::vector<std::size_t>& top_down) {

    std::size_t off = 0;
    for(std::size_t v : top_down) {
        if(!graph[v].state) continue;
        
        const std::size_t size = graph[v].state->getMatrixSize();

        graph[v].offset = off;
        graph[v].size = size;

        off += size;
    }

    return off;
}



// extend graph with compliance nodes
// precondition: all vertices have non-null state
static void extend_graph(graph_type& graph) {

    // find compliant nodes
    std::vector< std::size_t > compliant;
    for(std::size_t v : vertices(graph) ) {
        assert(graph[v].state);

        auto* node = node_cast(graph[v].state->getContext());
        if( node_compliance(node) ) {
            compliant.emplace_back(v);
        }
    }

    // create lambda + pairing nodes
    for(std::size_t c : compliant) {

        // lambda node
        vertex_type vprop;
        vprop.state = graph[c].state;
        
        const std::size_t v = add_vertex(vprop, graph);

        // pairing node
        const std::size_t p = add_vertex(graph);
        
        // pairing edges
        add_edge(p, v, graph);
        add_edge(p, c, graph);        
    }
    
};


static void concatenate(rmat& res, const rmat& src) {
    res.resize(src.rows(), src.cols());

    // TODO externalize these guys?
    std::vector<bool> mask(src.cols());
    std::vector<real> values(src.cols());
    std::vector<int> indices(src.cols());  
    
    // TODO better estimate? 
    const std::size_t estimated_nnz = src.nonZeros() + src.nonZeros() / 2;
    res.reserve(estimated_nnz);
    
    std::fill(mask.begin(), mask.end(), false);

    // compute result row-wise: res_0 = src; res_{i+1} = src * res_i;
    for(int i = 0, n = res.rows(); i < n; ++i) {
        std::size_t nnz = 0;

        // ith row in src
        for(rmat::InnerIterator src_it(src, i); src_it; ++src_it) {
            const real y = src_it.value();
            const int k = src_it.col();

            // take row from previously computed rows if possible
            const rmat& from = k < i ? res : src;
            
            for(rmat::InnerIterator res_it(from, k); res_it; ++res_it) {
                const int j = res_it.col();
                const real x = res_it.value();
                
                if(!mask[j]) {
                    mask[j] = true;
                    values[j] = x * y;
                    indices[nnz] = j;
                    ++nnz;
                } else {
                    values[j] += x * y;
                }
            }
        }

        std::sort(indices.data(), indices.data() + nnz);

        res.startVec(i);
        for(std::size_t k = 0; k < nnz; ++k) {
            const std::size_t j = indices[k];
            res.insertBack(i, j) = values[j];
            mask[j] = false;
        }
        
    }

    res.finalize();
}


template<class OutputIterator>
static void select_primal_dual(OutputIterator pout, std::size_t& p,
                               OutputIterator dout, std::size_t& d,
                               const graph_type& graph) {
    p = 0; d = 0;
    
    for(std::size_t v : vertices(graph)) {
        if(out_degree(v, graph) > 0) continue;

        assert(graph[v].state);
        
        // TODO store primal/dual status in stage3
        auto* node = node_cast(graph[v].state->getContext());

        if( node_compliance(node) ) {
            std::clog << "compliant: " << graph[v].size << " " << graph[v].offset << std::endl;
            // dual
            fill_identity_chunk(dout, graph[v].offset, d, graph[v].size);
            d += graph[v].size;
        } else {
            // primal

            // TODO optionally include bilaterals
            // TODO exclude projected dofs
            fill_identity_chunk(pout, graph[v].offset, p, graph[v].size);
            p += graph[v].size;            
        }
        
    }
    
}

system_type assemble_system(core::objectmodel::BaseContext* ctx,
                            const core::MechanicalParams* mp) {

    graph_type graph = create_graph(ctx, mp);

    // extend graph with lambda/pairing nodes
    extend_graph(graph);
    std::clog << "graph extended" << std::endl;
    
    // order graph
    std::vector<std::size_t> top_down;
    topological_sort(top_down, graph);

    // fill offset/size for vertices
    const std::size_t size = number_vertices(graph, top_down);
    std::clog << "graph numbered" << std::endl;
    
    // obtain mapping chunks
    triplets_type Js;
    fill_mapping(std::back_inserter(Js), graph);
    std::clog << "mappings fetched" << std::endl;

    // obtain mass/stiffness chunks
    triplets_type Hs;

    fill_forcefield(std::back_inserter(Hs), graph, mp);
    std::clog << "forcefields fetched" << std::endl;
    
    fill_compliance(std::back_inserter(Hs), graph, mp);
    std::clog << "compliance fetched" << std::endl;
    
    // build actual matrices
    rmat J(size, size);
    J.setFromTriplets(Js.begin(), Js.end());
    std::clog << "J:\n" << J << std::endl;
    
    rmat L;
    concatenate(L, J);
    std::clog << "mappings concatenated" << std::endl;
    std::clog << "L:\n" << L << std::endl;
    
    rmat H(size, size);
    H.setFromTriplets(Hs.begin(), Hs.end());
    std::clog << "H:\n" << H << std::endl;    

    // primal/dual selection
    triplets_type Ps, Ds;
    std::size_t p, d;
    select_primal_dual(std::back_inserter(Ps), p, std::back_inserter(Ds), d, graph);
    std::clog << "primal/dual selected: " << p << " / " << d << std::endl;

    // TODO fetch projectors    
    
    // TODO which side is the fastest?
    const rmat K = ((L.transpose() * H) * L).triangularView<Eigen::Lower>();
    std::clog << "K:\n" << K << std::endl;

    system_type res;

    if( p ) {
        rmat P(size, p); P.setFromTriplets(Ps.begin(), Ps.end());
        std::clog << "P:\n" << P << std::endl;

        // TODO optimize selection        
        res.H = ((P.transpose() * K) * P).triangularView<Eigen::Lower>();
        std::clog << "res.H\n" << res.H << std::endl;
    

        if( d ) {
            rmat D(size, d); D.setFromTriplets(Ds.begin(), Ds.end());
            std::clog << "D:\n" << D << std::endl;
            
            // TODO optimize selection
            res.J = (D.transpose() * K) * P;
            std::clog << "res.J\n" << res.J << std::endl;        

            // TODO optimize selection
            res.C = ((D.transpose() * K) * D).triangularView<Eigen::Lower>();
            std::clog << "res.C\n" << res.C << std::endl;                
        }
    }
    
    return res;
}


}
}
