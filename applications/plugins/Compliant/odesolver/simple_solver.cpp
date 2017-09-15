#include "simple_solver.hpp"

#include <sofa/core/ObjectFactory.h>

// #include <SofaEigen2Solver/EigenSparseMatrix.h>

// #include <sofa/simulation/PropagateEventVisitor.h>

// #include "../assembly/AssemblyVisitor.h"
// #include "../utils/scoped.h"

#include <Compliant/assembly/assembly.hpp>

namespace sofa {
namespace component {
namespace odesolver {

SOFA_DECL_CLASS(simple_solver);

static int handle = core::RegisterObject("a simple implicit assembled solver")
    .add< simple_solver >();


simple_solver::simple_solver()
    : stabilization(initData(&stabilization, true, "stabilization", "constraint stabilization")),
      debug(initData(&debug, false, "debug", "print debug stuff")),
      constraint_forces(initData(&constraint_forces, false, "constraint_forces",
                                 "add constraint forces to mstate's 'force' vector after solve"))
{
    
}

simple_solver::~simple_solver() { }


static core::MechanicalParams mechanical_params(const core::ExecParams& ep, SReal dt) {
    core::MechanicalParams res(ep);

    res.setMFactor(1.0);
    res.setBFactor(dt);
    res.setKFactor(dt * dt);        
    
    return res;
}

void simple_solver::solve(const core::ExecParams* ep,
                          SReal dt,
                          core::MultiVecCoordId posId,
                          core::MultiVecDerivId velId) {
    check();        // TODO this should be called automatically from the outside

    auto assembler = assembly::make_assembler();
    assembler->init(getContext());

    const core::MechanicalParams mparams = mechanical_params(*ep, dt);
    const assembly::system_type sys = assembler->assemble_system(&mparams);
    
}


struct kkt_error : std::runtime_error {
    kkt_error() : std::runtime_error("no KKT Solver found") { }
};


void simple_solver::check() {
    if( !kkt ) throw kkt_error();
}

void simple_solver::init() {
    kkt = getContext()->get<kkt_type>(core::objectmodel::BaseContext::Local);

    check();        // TODO this should be called automatically from the outside
}







}
}
}
