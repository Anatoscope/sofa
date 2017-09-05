#ifndef SOFA_COMPLIANT_ASSEMBLY_HPP
#define SOFA_COMPLIANT_ASSEMBLY_HPP

#include <Compliant/assembly/AssembledSystem.h>

namespace sofa {
namespace component {

void assemble(linearsolver::AssembledSystem& res,
              core::objectmodel::BaseContext* ctx,
              const core::MechanicalParams *mparams);

}
}

                


#endif
