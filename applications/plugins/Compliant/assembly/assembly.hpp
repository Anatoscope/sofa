#ifndef SOFA_COMPLIANT_ASSEMBLY_HPP
#define SOFA_COMPLIANT_ASSEMBLY_HPP

#include <Compliant/assembly/AssembledSystem.h>
#include <memory>

namespace sofa {
namespace assembly {

using system_type = component::linearsolver::AssembledSystem;


struct assembler_base {
    virtual ~assembler_base() { }

    virtual void init(core::objectmodel::BaseContext* ctx) = 0;
    
    virtual system_type assemble_system(const core::MechanicalParams *mparams) = 0;

    virtual void rhs_dynamics(system_type::vec& out, const core::MechanicalParams *mparams) const = 0;
    virtual void rhs_correction(system_type::vec& out, const core::MechanicalParams *mparams) const = 0;

    virtual void integrate(const system_type::vec& v, system_type::real dt) const = 0;
};


std::unique_ptr<assembler_base> make_assembler();


system_type assemble_system(core::objectmodel::BaseContext* ctx, const core::MechanicalParams *mparams);
}
}

                


#endif
