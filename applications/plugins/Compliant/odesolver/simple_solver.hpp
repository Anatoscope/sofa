#ifndef COMPLIANT_SIMPLE_SOLVER_HPP
#define COMPLIANT_SIMPLE_SOLVER_HPP

#include <Compliant/config.h>

#include <sofa/core/behavior/OdeSolver.h>
#include <Compliant/numericalsolver/KKTSolver.h>

namespace sofa {
namespace component {
namespace odesolver {

class SOFA_Compliant_API simple_solver : public sofa::core::behavior::OdeSolver {
    using kkt_type = linearsolver::KKTSolver;
    kkt_type::SPtr kkt;
 public:
	SOFA_CLASS(simple_solver, sofa::core::behavior::OdeSolver);

    virtual void check();
    virtual void init();
    
    virtual void solve(const core::ExecParams* params,
                       SReal dt,
                       core::MultiVecCoordId posId,
                       core::MultiVecDerivId velId);
    
	simple_solver();
    ~simple_solver();

    Data<bool> stabilization, debug, constraint_forces;
    
};




}
}
}



#endif
