#include "UniformCompliance.h"
#include <iostream>

using std::cerr;
using std::endl;

namespace sofa
{
namespace component
{
namespace forcefield
{


template<class DataTypes>
const typename UniformCompliance<DataTypes>::Real
UniformCompliance<DataTypes>::epsilon = std::numeric_limits<Real>::epsilon();


template<class DataTypes>
UniformCompliance<DataTypes>::UniformCompliance( core::behavior::MechanicalState<DataTypes> *mm )
    : Inherit(mm)
    , compliance( initData(&compliance, Real(0), "compliance", 
                           "Compliance value uniformly applied to all the DOF."))
    , damping( initData(&damping, Real(0), "damping", "uniform viscous damping."))
	  
{
    this->isCompliance.setValue(true);
}

template<class DataTypes>
void UniformCompliance<DataTypes>::init()
{
    Inherit::init();
    if( !this->getMState() ) {
        msg_error() << "no mechanical state";
    } else {
        reinit();
    }
}

template<class DataTypes>
void UniformCompliance<DataTypes>::reinit()
{
    core::behavior::BaseMechanicalState* state = this->getContext()->getMechanicalState();
    assert(state);

    const Real c = compliance.getValue();

    if( this->isCompliance.getValue() ) {
        matC.resize(state->getMatrixSize(), state->getMatrixSize());

        if( c ) {
            
            for(unsigned i = 0, n = state->getMatrixSize(); i < n; i++) {
                matC.beginRow(i);
                matC.insertBack(i, i, c);
            }

            matC.finalize();
        }

        matK.compressedMatrix.resize(0,0);        
    } else {
        const Real clamped_c = std::max<Real>(c, epsilon);

        if(clamped_c == epsilon) {
            msg_warning() << "clamped compliance to avoid infinite stiffness";
        }
        
        const Real k = -1 / clamped_c;
        
        matK.resize(state->getMatrixSize(), state->getMatrixSize());

        for(unsigned i = 0, n = state->getMatrixSize(); i < n; ++i) {
            matK.beginRow(i);
            matK.insertBack(i, i, k);
        }
        
        matK.finalize();
        
        matC.compressedMatrix.resize(0,0);
    }
    
    if( this->rayleighStiffness.getValue() ) {
        msg_warning() << "rayleighStiffness ignored, use 'damping' instead";
    }
    
    
	if( damping.getValue() > 0 ) {
        const Real& d = damping.getValue();

		matB.resize(state->getMatrixSize(), state->getMatrixSize());
		
		for(unsigned i=0, n = state->getMatrixSize(); i < n; i++) {
            matB.beginRow(i);
            matB.insertBack(i, i, -d);
		}

        matB.finalize();
	} else {
        matB.compressedMatrix.resize(0,0);
    }
	
}

template<class DataTypes>
SReal UniformCompliance<DataTypes>::getPotentialEnergy( const core::MechanicalParams* /*mparams*/, const DataVecCoord& x ) const
{
    const VecCoord& _x = x.getValue();
    const unsigned m = this->mstate->getMatrixBlockSize();

    const Real c = compliance.getValue();
    
    const Real clamped_c = std::max<Real>(c, epsilon);
    const Real k = 1 / clamped_c;

    // TODO this makes little to no sense when c is that small
    SReal e = 0;
    for( unsigned int i=0, n = _x.size(); i < n ; ++i ){
        for( unsigned int j=0 ; j<m ; ++j ) {
            e += .5 * k * _x[i][j] * _x[i][j];
        }
    }
    return e;
}


template<class DataTypes>
const sofa::defaulttype::BaseMatrix* UniformCompliance<DataTypes>::getComplianceMatrix(const core::MechanicalParams*) {
    if(this->isCompliance.getValue() ) {
        if( this->mstate->getSize() != std::size_t(matC.rows()) ) {
            reinit();
        }
    
        return &matC;
    }

    return nullptr;
}


template<class DataTypes>
void UniformCompliance<DataTypes>::addKToMatrix( sofa::defaulttype::BaseMatrix * matrix, SReal kFact, unsigned int &offset )
{
  if(!this->isCompliance.getValue() ) {
    matK.addToBaseMatrix( matrix, kFact, offset );
  }
}

template<class DataTypes>
void UniformCompliance<DataTypes>::addBToMatrix( sofa::defaulttype::BaseMatrix * matrix, SReal bFact, unsigned int &offset )
{
	if( damping.getValue() > 0 ) { // B is empty in that case
		matB.addToBaseMatrix( matrix, bFact, offset );
	}
}

template<class DataTypes>
void UniformCompliance<DataTypes>::addForce(const core::MechanicalParams *, DataVecDeriv& f, const DataVecCoord& x, const DataVecDeriv& /*v*/) {

    if(!this->isCompliance.getValue()) {
        if( x.getValue().size() != std::size_t(matK.compressedMatrix.rows()) )  {
            reinit();
        }
        
        matK.addMult(f, x);
    }
}

template<class DataTypes>
void UniformCompliance<DataTypes>::addDForce(const core::MechanicalParams *mparams, DataVecDeriv& df, const DataVecDeriv& dx) {
    // mtournier: wtf? the rayleihtStiffness should be used to generate the damping matrix
    // const Real kfactor = mparams->kFactorIncludingRayleighDamping(this->rayleighStiffness.getValue());
    const Real kfactor = mparams->kFactor();
    
    if( kfactor && !this->isCompliance.getValue() ) {
        matK.addMult( df, dx, kfactor );
    }

    if( damping.getValue() > 0 ) {
        const Real bfactor = mparams->bFactor();
        if(bfactor) matB.addMult( df, dx, bfactor );
    }
}

template<class DataTypes>
void UniformCompliance<DataTypes>::addClambda(const core::MechanicalParams *, DataVecDeriv &res, const DataVecDeriv &lambda, SReal cfactor)
{
    matC.addMult( res, lambda, cfactor );
}


}
}
}
