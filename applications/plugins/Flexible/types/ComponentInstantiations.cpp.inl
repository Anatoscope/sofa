

#include <Flexible/config.h>


#include "ComponentSpecializationsDefines.h"

#include <SofaBoundaryCondition/ProjectToPointConstraint.inl>
#include <SofaBoundaryCondition/ProjectToLineConstraint.inl>
#include <SofaBoundaryCondition/ProjectToPlaneConstraint.inl>
#include <SofaBoundaryCondition/ProjectDirectionConstraint.inl>

#include <sofa/core/ObjectFactory.h>

#include <sofa/core/State.inl>
#include <SofaBaseMechanics/MechanicalObject.inl>

#include <SofaBoundaryCondition/FixedConstraint.inl>
#include <SofaBoundaryCondition/PartialFixedConstraint.inl>
#include <sofa/core/behavior/ProjectiveConstraintSet.inl>

#include <SofaEngine/BoxROI.inl>


#include <SofaBaseMechanics/UniformMass.inl>

#include <SofaValidation/Monitor.inl>
#include <SofaValidation/ExtraMonitor.inl>

#include <SofaConstraint/UncoupledConstraintCorrection.inl>

#include <SofaBaseMechanics/IdentityMapping.inl>
#include <SofaMiscMapping/SubsetMultiMapping.inl>

#include <sofa/core/behavior/ForceField.inl>
#include <sofa/core/behavior/Mass.inl>
#include <sofa/core/behavior/ConstraintCorrection.inl>
#include <SofaDeformable/RestShapeSpringsForceField.inl>
#include <SofaBoundaryCondition/ConstantForceField.inl>
#include <SofaBoundaryCondition/UniformVelocityDampingForceField.inl>


#ifdef SOFA_HAVE_IMAGE
#include "../mass/ImageDensityMass.inl"
#endif


#include <sofa/core/Mapping.inl>
#include <sofa/core/MultiMapping.inl>


namespace sofa
{

namespace core
{


#ifndef SOFA_FLOAT
    template class SOFA_Flexible_API State< defaulttype::TYPEABSTRACTNAME3dTypes >;
    template class SOFA_Flexible_API Mapping< defaulttype::TYPEABSTRACTNAME3dTypes, defaulttype::Vec3dTypes >;
    template class SOFA_Flexible_API Mapping< defaulttype::TYPEABSTRACTNAME3dTypes, defaulttype::Rigid3dTypes >;
    template class SOFA_Flexible_API MultiMapping< defaulttype::TYPEABSTRACTNAME3dTypes, defaulttype::TYPEABSTRACTNAME3dTypes >;
#endif
#ifndef SOFA_DOUBLE
    template class SOFA_Flexible_API State< defaulttype::TYPEABSTRACTNAME3fTypes >;
    template class SOFA_Flexible_API Mapping< defaulttype::TYPEABSTRACTNAME3fTypes, defaulttype::Vec3fTypes >;
    template class SOFA_Flexible_API Mapping< defaulttype::TYPEABSTRACTNAME3fTypes, defaulttype::Rigid3fTypes >;
    template class SOFA_Flexible_API MultiMapping< defaulttype::TYPEABSTRACTNAME3fTypes, defaulttype::TYPEABSTRACTNAME3fTypes >;
#endif


namespace behavior
{


#ifndef SOFA_FLOAT
    template class SOFA_Flexible_API ForceField< defaulttype::TYPEABSTRACTNAME3dTypes >;
    template class SOFA_Flexible_API Mass< defaulttype::TYPEABSTRACTNAME3dTypes >;
    template class SOFA_Flexible_API ConstraintCorrection< defaulttype::TYPEABSTRACTNAME3dTypes >;
    template class SOFA_Flexible_API ProjectiveConstraintSet< defaulttype::TYPEABSTRACTNAME3dTypes >;
#endif
#ifndef SOFA_DOUBLE
    template class SOFA_Flexible_API ForceField< defaulttype::TYPEABSTRACTNAME3fTypes >;
    template class SOFA_Flexible_API Mass< defaulttype::TYPEABSTRACTNAME3fTypes >;
    template class SOFA_Flexible_API ConstraintCorrection< defaulttype::TYPEABSTRACTNAME3fTypes >;
    template class SOFA_Flexible_API ProjectiveConstraintSet< defaulttype::TYPEABSTRACTNAME3fTypes >;
#endif


} // namespace behavior

} // namespace core



namespace component
{
namespace projectiveconstraintset
{

using namespace sofa::defaulttype;
using namespace sofa::helper;



// ==========================================================================
// FixedConstraint



SOFA_DECL_CLASS ( EVALUATOR(TYPEABSTRACTNAME,FixedConstraint) )
int EVALUATOR(TYPEABSTRACTNAME,FixedConstraintClass) = core::RegisterObject ( "Attach given dofs to their initial positions" )
#ifndef SOFA_FLOAT
    .add< FixedConstraint<defaulttype::TYPEABSTRACTNAME3dTypes> >()
#endif
#ifndef SOFA_DOUBLE
    .add< FixedConstraint<defaulttype::TYPEABSTRACTNAME3fTypes> >()
#endif
    ;
#ifndef SOFA_FLOAT
template class SOFA_Flexible_API FixedConstraint<TYPEABSTRACTNAME3dTypes>;
#endif
#ifndef SOFA_DOUBLE
template class SOFA_Flexible_API FixedConstraint<TYPEABSTRACTNAME3fTypes>;
#endif







// ==========================================================================
// PartialFixedConstraint



SOFA_DECL_CLASS ( EVALUATOR(TYPEABSTRACTNAME,PartialFixedConstraint) )
int EVALUATOR(TYPEABSTRACTNAME,PartialFixedConstraintClass) = core::RegisterObject ( "Attach given cinematic dofs to their initial positions" )
#ifndef SOFA_FLOAT
    .add< PartialFixedConstraint<defaulttype::TYPEABSTRACTNAME3dTypes> >()
#endif
#ifndef SOFA_DOUBLE
    .add< PartialFixedConstraint<defaulttype::TYPEABSTRACTNAME3fTypes> >()
#endif
    ;
#ifndef SOFA_FLOAT
template class SOFA_Flexible_API PartialFixedConstraint<TYPEABSTRACTNAME3dTypes>;
#endif
#ifndef SOFA_DOUBLE
template class SOFA_Flexible_API PartialFixedConstraint<TYPEABSTRACTNAME3fTypes>;
#endif




// ==========================================================================
// ProjectToPointConstraint
SOFA_DECL_CLASS ( EVALUATOR(TYPEABSTRACTNAME,ProjectToPointConstraint) )
int EVALUATOR(TYPEABSTRACTNAME,ProjectToPointConstraintClass) = core::RegisterObject ( "Project particles to a point" )
#ifndef SOFA_FLOAT
        .add< ProjectToPointConstraint<defaulttype::TYPEABSTRACTNAME3dTypes> >()
#endif
#ifndef SOFA_DOUBLE
.add< ProjectToPointConstraint<defaulttype::TYPEABSTRACTNAME3fTypes> >()
#endif
        ;
#ifndef SOFA_FLOAT
template class SOFA_Flexible_API ProjectToPointConstraint<TYPEABSTRACTNAME3dTypes>;
#endif
#ifndef SOFA_DOUBLE
template class SOFA_Flexible_API ProjectToPointConstraint<TYPEABSTRACTNAME3fTypes>;
#endif




} // namespace projectiveconstraintset
} // namespace component
} // namespace sofa



#include <sofa/helper/gl/Axis.h>
namespace sofa
{
namespace component
{
namespace container
{

using defaulttype::Vector3;
using defaulttype::Quat;
using defaulttype::Vec4f;

// ==========================================================================
// Instanciation

SOFA_DECL_CLASS ( EVALUATOR(TYPEABSTRACTNAME,MechanicalObject) )

using namespace sofa::defaulttype;

int EVALUATOR(TYPEABSTRACTNAME,MechanicalObjectClass) = core::RegisterObject ( "mechanical state vectors" )
    #ifndef SOFA_FLOAT
        .add< MechanicalObject<TYPEABSTRACTNAME3dTypes> >()
    #endif
    #ifndef SOFA_DOUBLE
        .add< MechanicalObject<TYPEABSTRACTNAME3fTypes> >()
    #endif
        ;



#ifndef SOFA_FLOAT
    template class SOFA_Flexible_API MechanicalObject<TYPEABSTRACTNAME3dTypes>;
#endif
#ifndef SOFA_DOUBLE
    template class SOFA_Flexible_API MechanicalObject<TYPEABSTRACTNAME3fTypes>;
#endif



} // namespace container

namespace mass
{



// ==========================================================================
// Instanciation

using namespace sofa::defaulttype;

SOFA_DECL_CLASS ( EVALUATOR(TYPEABSTRACTNAME,UniformMass) )

int EVALUATOR(TYPEABSTRACTNAME,UniformMassClass) = core::RegisterObject ( "Define the same mass for all the particles" )
#ifndef SOFA_FLOAT
    .add< UniformMass<TYPEABSTRACTNAME3dTypes,TYPEABSTRACTNAME3dMass> >()
#endif
#ifndef SOFA_DOUBLE
    .add< UniformMass<TYPEABSTRACTNAME3fTypes,TYPEABSTRACTNAME3fMass> >()
#endif
    ;


#ifdef SOFA_HAVE_IMAGE

SOFA_DECL_CLASS ( EVALUATOR(TYPEABSTRACTNAME,ImageDensityMass) )

int EVALUATOR(TYPEABSTRACTNAME,ImageDensityMassClass) = core::RegisterObject ( "Define a global mass matrix including non diagonal terms" )
#ifndef SOFA_FLOAT
    .add< ImageDensityMass<TYPEABSTRACTNAME3dTypes,core::behavior::ShapeFunctiond,TYPEABSTRACTNAME3dMass> >()
#endif
#ifndef SOFA_DOUBLE
    .add< ImageDensityMass<TYPEABSTRACTNAME3fTypes,core::behavior::ShapeFunctionf,TYPEABSTRACTNAME3fMass> >()
#endif
    ;


#ifndef SOFA_FLOAT
template class SOFA_Flexible_API ImageDensityMass<TYPEABSTRACTNAME3dTypes,core::behavior::ShapeFunctiond,TYPEABSTRACTNAME3dMass>;
#endif
#ifndef SOFA_DOUBLE
template class SOFA_Flexible_API ImageDensityMass<TYPEABSTRACTNAME3fTypes,core::behavior::ShapeFunctionf,TYPEABSTRACTNAME3fMass>;
#endif


#endif


#ifndef SOFA_FLOAT
template class SOFA_Flexible_API UniformMass<TYPEABSTRACTNAME3dTypes,TYPEABSTRACTNAME3dMass>;
#endif
#ifndef SOFA_DOUBLE
template class SOFA_Flexible_API UniformMass<TYPEABSTRACTNAME3fTypes,TYPEABSTRACTNAME3fMass>;
#endif


} // namespace mass

namespace misc
{


SOFA_DECL_CLASS( EVALUATOR(TYPEABSTRACTNAME,Monitor) )
// Register in the Factory
int EVALUATOR(TYPEABSTRACTNAME,MonitorClass) = core::RegisterObject("Monitoring of particles")
#ifndef SOFA_FLOAT
        .add< Monitor<defaulttype::TYPEABSTRACTNAME3dTypes> >()
#endif
#ifndef SOFA_DOUBLE
        .add< Monitor<defaulttype::TYPEABSTRACTNAME3fTypes> >()
#endif
    ;

#ifndef SOFA_FLOAT
    template class SOFA_Flexible_API Monitor<defaulttype::TYPEABSTRACTNAME3dTypes>;
#endif
#ifndef SOFA_DOUBLE
    template class SOFA_Flexible_API Monitor<defaulttype::TYPEABSTRACTNAME3fTypes>;
#endif





SOFA_DECL_CLASS( EVALUATOR(TYPEABSTRACTNAME,ExtraMonitor) )
// Register in the Factory
int EVALUATOR(TYPEABSTRACTNAME,ExtraMonitorClass) = core::RegisterObject("Monitoring of particles")
#ifndef SOFA_FLOAT
    .add< ExtraMonitor<defaulttype::TYPEABSTRACTNAME3dTypes> >()
#endif
#ifndef SOFA_DOUBLE
    .add< ExtraMonitor<defaulttype::TYPEABSTRACTNAME3fTypes> >()
#endif
;



#ifndef SOFA_FLOAT
    template class SOFA_Flexible_API ExtraMonitor<defaulttype::TYPEABSTRACTNAME3dTypes>;
#endif
#ifndef SOFA_DOUBLE
    template class SOFA_Flexible_API ExtraMonitor<defaulttype::TYPEABSTRACTNAME3fTypes>;
#endif



} // namespace misc

namespace constraintset
{
SOFA_DECL_CLASS( EVALUATOR(TYPEABSTRACTNAME,UncoupledConstraintCorrection) )
// Register in the Factory
int EVALUATOR(TYPEABSTRACTNAME,UncoupledConstraintCorrectionClass) = core::RegisterObject("Component computing contact forces within a simulated body using the compliance method.")
#ifndef SOFA_FLOAT
    .add< UncoupledConstraintCorrection<defaulttype::TYPEABSTRACTNAME3dTypes> >()
#endif
#ifndef SOFA_DOUBLE
    .add< UncoupledConstraintCorrection<defaulttype::TYPEABSTRACTNAME3fTypes> >()
#endif
        ;

#ifndef SOFA_FLOAT
    template class SOFA_Flexible_API UncoupledConstraintCorrection<defaulttype::TYPEABSTRACTNAME3dTypes>;
#endif
#ifndef SOFA_DOUBLE
    template class SOFA_Flexible_API UncoupledConstraintCorrection<defaulttype::TYPEABSTRACTNAME3fTypes>;
#endif


} // namespace constraintset

namespace mapping
{


SOFA_DECL_CLASS(EVALUATOR(TYPEABSTRACTNAME,IdentityMapping))

// Register in the Factory
int EVALUATOR(TYPEABSTRACTNAME,IdentityMappingClass) = core::RegisterObject("Special case of mapping where the child points are the same as the parent points")
#ifndef SOFA_FLOAT
    .add< IdentityMapping< defaulttype::TYPEABSTRACTNAME3dTypes, defaulttype::Vec3dTypes > >()
    .add< IdentityMapping< defaulttype::TYPEABSTRACTNAME3dTypes, defaulttype::ExtVec3dTypes > >()
    .add< IdentityMapping< defaulttype::TYPEABSTRACTNAME3dTypes, defaulttype::TYPEABSTRACTNAME3dTypes > >()
#endif
#ifndef SOFA_DOUBLE
    .add< IdentityMapping< defaulttype::TYPEABSTRACTNAME3fTypes, defaulttype::Vec3fTypes > >()
    .add< IdentityMapping< defaulttype::TYPEABSTRACTNAME3fTypes, defaulttype::ExtVec3fTypes > >()
    .add< IdentityMapping< defaulttype::TYPEABSTRACTNAME3fTypes, defaulttype::TYPEABSTRACTNAME3fTypes > >()
#endif
#ifndef SOFA_FLOAT
#ifndef SOFA_DOUBLE
    .add< IdentityMapping< defaulttype::TYPEABSTRACTNAME3fTypes, defaulttype::Vec3dTypes > >()
    .add< IdentityMapping< defaulttype::TYPEABSTRACTNAME3fTypes, defaulttype::ExtVec3dTypes > >()
    .add< IdentityMapping< defaulttype::TYPEABSTRACTNAME3dTypes, defaulttype::Vec3fTypes > >()
    .add< IdentityMapping< defaulttype::TYPEABSTRACTNAME3dTypes, defaulttype::ExtVec3fTypes > >()
    .add< IdentityMapping< defaulttype::TYPEABSTRACTNAME3fTypes, defaulttype::TYPEABSTRACTNAME3dTypes > >()
    .add< IdentityMapping< defaulttype::TYPEABSTRACTNAME3dTypes, defaulttype::TYPEABSTRACTNAME3fTypes > >()
#endif
#endif
    ;




#ifndef SOFA_FLOAT
template class SOFA_Flexible_API IdentityMapping< defaulttype::TYPEABSTRACTNAME3dTypes, defaulttype::Vec3dTypes >;
template class SOFA_Flexible_API IdentityMapping< defaulttype::TYPEABSTRACTNAME3dTypes, defaulttype::ExtVec3dTypes >;
template class SOFA_Flexible_API IdentityMapping< defaulttype::TYPEABSTRACTNAME3dTypes, defaulttype::TYPEABSTRACTNAME3dTypes >;
#endif
#ifndef SOFA_DOUBLE
template class SOFA_Flexible_API IdentityMapping< defaulttype::TYPEABSTRACTNAME3fTypes, defaulttype::Vec3fTypes >;
template class SOFA_Flexible_API IdentityMapping< defaulttype::TYPEABSTRACTNAME3fTypes, defaulttype::ExtVec3fTypes >;
template class SOFA_Flexible_API IdentityMapping< defaulttype::TYPEABSTRACTNAME3fTypes, defaulttype::TYPEABSTRACTNAME3fTypes >;
#endif
#ifndef SOFA_FLOAT
#ifndef SOFA_DOUBLE
template class SOFA_Flexible_API IdentityMapping< defaulttype::TYPEABSTRACTNAME3fTypes, defaulttype::Vec3dTypes >;
template class SOFA_Flexible_API IdentityMapping< defaulttype::TYPEABSTRACTNAME3fTypes, defaulttype::ExtVec3dTypes >;
template class SOFA_Flexible_API IdentityMapping< defaulttype::TYPEABSTRACTNAME3dTypes, defaulttype::Vec3fTypes >;
template class SOFA_Flexible_API IdentityMapping< defaulttype::TYPEABSTRACTNAME3dTypes, defaulttype::ExtVec3fTypes >;
template class SOFA_Flexible_API IdentityMapping< defaulttype::TYPEABSTRACTNAME3fTypes, defaulttype::TYPEABSTRACTNAME3dTypes >;
template class SOFA_Flexible_API IdentityMapping< defaulttype::TYPEABSTRACTNAME3dTypes, defaulttype::TYPEABSTRACTNAME3fTypes >;
#endif
#endif



///////////////////////////////

using namespace sofa::defaulttype;

SOFA_DECL_CLASS(EVALUATOR(TYPEABSTRACTNAME,SubsetMultiMapping))

int EVALUATOR(TYPEABSTRACTNAME,SubsetMultiMappingClass) = core::RegisterObject("Compute a subset of the input MechanicalObjects according to a dof index list")
#ifndef SOFA_FLOAT
    .add< SubsetMultiMapping< TYPEABSTRACTNAME3dTypes, TYPEABSTRACTNAME3dTypes > >()
#endif
#ifndef SOFA_DOUBLE
    .add< SubsetMultiMapping< TYPEABSTRACTNAME3fTypes, TYPEABSTRACTNAME3fTypes > >()
#endif
    ;

#ifndef SOFA_FLOAT
template class SOFA_Flexible_API SubsetMultiMapping< TYPEABSTRACTNAME3dTypes, TYPEABSTRACTNAME3dTypes >;
#endif
#ifndef SOFA_DOUBLE
template class SOFA_Flexible_API SubsetMultiMapping< TYPEABSTRACTNAME3fTypes, TYPEABSTRACTNAME3fTypes >;
#endif


} // namespace mapping


namespace engine
{
SOFA_DECL_CLASS(EVALUATOR(TYPEABSTRACTNAME,BoxROI))

// Register in the Factory
int EVALUATOR(TYPEABSTRACTNAME,BoxROIClass) = core::RegisterObject("Find the primitives (vertex/edge/triangle/tetrahedron) inside a given box")
#ifndef SOFA_FLOAT
    .add< BoxROI< defaulttype::TYPEABSTRACTNAME3dTypes > >()
#endif
#ifndef SOFA_DOUBLE
    .add< BoxROI< defaulttype::TYPEABSTRACTNAME3fTypes > >()
#endif
    ;


#ifndef SOFA_FLOAT
template class SOFA_Flexible_API boxroi::BoxROI< defaulttype::TYPEABSTRACTNAME3dTypes >;
#endif
#ifndef SOFA_DOUBLE
template class SOFA_Flexible_API boxroi::BoxROI< defaulttype::TYPEABSTRACTNAME3fTypes >;
#endif

} // namespace engine

namespace forcefield
{

SOFA_DECL_CLASS(EVALUATOR(TYPEABSTRACTNAME,RestShapeSpringsForceField))

// Register in the Factory
int EVALUATOR(TYPEABSTRACTNAME,RestShapeSpringsForceFieldClass) = core::RegisterObject("Spring attached to rest position")
#ifndef SOFA_FLOAT
    .add< RestShapeSpringsForceField< defaulttype::TYPEABSTRACTNAME3dTypes > >()
#endif
#ifndef SOFA_DOUBLE
    .add< RestShapeSpringsForceField< defaulttype::TYPEABSTRACTNAME3fTypes > >()
#endif
    ;

#ifndef SOFA_FLOAT
template class SOFA_Flexible_API RestShapeSpringsForceField< defaulttype::TYPEABSTRACTNAME3dTypes >;
#endif
#ifndef SOFA_DOUBLE
template class SOFA_Flexible_API RestShapeSpringsForceField< defaulttype::TYPEABSTRACTNAME3fTypes >;
#endif





SOFA_DECL_CLASS(EVALUATOR(TYPEABSTRACTNAME,ConstantForceField))

// Register in the Factory
int EVALUATOR(TYPEABSTRACTNAME,ConstantForceFieldClass) = core::RegisterObject("Constant forces applied to given degrees of freedom")
#ifndef SOFA_FLOAT
    .add< ConstantForceField< defaulttype::TYPEABSTRACTNAME3dTypes > >()
#endif
#ifndef SOFA_DOUBLE
    .add< ConstantForceField< defaulttype::TYPEABSTRACTNAME3fTypes > >()
#endif
    ;

#ifndef SOFA_FLOAT
template class SOFA_Flexible_API ConstantForceField< defaulttype::TYPEABSTRACTNAME3dTypes >;
#endif
#ifndef SOFA_DOUBLE
template class SOFA_Flexible_API ConstantForceField< defaulttype::TYPEABSTRACTNAME3fTypes >;
#endif

SOFA_DECL_CLASS(EVALUATOR(TYPEABSTRACTNAME,UniformVelocityDampingForceField))

// Register in the Factory
int EVALUATOR(TYPEABSTRACTNAME,UniformVelocityDampingForceFieldClass) = core::RegisterObject("Uniform velocity damping")
#ifndef SOFA_FLOAT
    .add< UniformVelocityDampingForceField< defaulttype::TYPEABSTRACTNAME3dTypes > >()
#endif
#ifndef SOFA_DOUBLE
    .add< UniformVelocityDampingForceField< defaulttype::TYPEABSTRACTNAME3fTypes > >()
#endif
    ;

#ifndef SOFA_FLOAT
template class SOFA_Flexible_API UniformVelocityDampingForceField<defaulttype::TYPEABSTRACTNAME3dTypes>;
#endif
#ifndef SOFA_DOUBLE
template class SOFA_Flexible_API UniformVelocityDampingForceField<defaulttype::TYPEABSTRACTNAME3fTypes>;
#endif

} // namespace forcefield

} // namespace component



} // namespace sofa


#include "ComponentSpecializationsUndef.h"
