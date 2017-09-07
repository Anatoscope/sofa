#ifndef SOFA_COMPLIANT_ASSEMBLY_HPP
#define SOFA_COMPLIANT_ASSEMBLY_HPP

#include <Compliant/assembly/AssembledSystem.h>

namespace sofa {
namespace assembly {

using system_type = component::linearsolver::AssembledSystem;

system_type assemble_system(core::objectmodel::BaseContext* ctx,
                            const core::MechanicalParams *mparams);

}
}

                


#endif
