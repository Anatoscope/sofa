

#include <Flexible/config.h>


#include <SofaBaseMechanics/MechanicalObject.h>


#include <SofaBaseMechanics/UniformMass.h>

#include <sofa/core/behavior/ForceField.h>
#include <sofa/core/behavior/Mass.h>
#include <sofa/core/behavior/ProjectiveConstraintSet.h>
#include <sofa/core/behavior/ConstraintCorrection.h>
#include <sofa/core/Mapping.h>


#ifdef SOFA_HAVE_IMAGE
#include "../mass/ImageDensityMass.h"
#endif

// working around this macro hell

#include "ComponentSpecializationsDefines.h"

#define DONT_UNDEF_FFS
#include "ComponentSpecializations.h.inl"
#undef DONT_UNDEF_FFS






namespace sofa
{


// ==========================================================================
// Mechanical Object

namespace component
{
namespace container
{


#if defined(SOFA_EXTERN_TEMPLATE) && !defined(FLEXIBLE_COMPILING_CPP)
#ifndef SOFA_FLOAT
extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::TYPEABSTRACTNAME3dTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::TYPEABSTRACTNAME3dTypes>;
#endif
#ifndef SOFA_DOUBLE
extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::TYPEABSTRACTNAME3fTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::TYPEABSTRACTNAME3fTypes>;
#endif
#endif

} // namespace container







// ==========================================================================
// Uniform Mass


namespace mass
{


#if defined(SOFA_EXTERN_TEMPLATE) && !defined(FLEXIBLE_COMPILING_CPP)

    #ifndef SOFA_FLOAT
        #ifdef SOFA_HAVE_IMAGE
            extern template class SOFA_Flexible_API ImageDensityMass<defaulttype::TYPEABSTRACTNAME3dTypes,core::behavior::ShapeFunctiond,defaulttype::TYPEABSTRACTNAME3dMass>;
        #endif
        extern template class SOFA_Flexible_API UniformMass<defaulttype::TYPEABSTRACTNAME3dTypes,defaulttype::TYPEABSTRACTNAME3dMass>;
    #endif

    #ifndef SOFA_DOUBLE
        #ifdef SOFA_HAVE_IMAGE
            extern template class SOFA_Flexible_API ImageDensityMass<defaulttype::TYPEABSTRACTNAME3fTypes,core::behavior::ShapeFunctionf,defaulttype::TYPEABSTRACTNAME3fMass>;
        #endif
        extern template class SOFA_Flexible_API UniformMass<defaulttype::TYPEABSTRACTNAME3fTypes,defaulttype::TYPEABSTRACTNAME3fMass>;
    #endif

#endif



} // namespace mass



} // namespace component



namespace core
{

namespace behavior
{

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(FLEXIBLE_COMPILING_CPP)
#ifndef SOFA_FLOAT
extern template class SOFA_Flexible_API ForceField<defaulttype::TYPEABSTRACTNAME3dTypes>;
extern template class SOFA_Flexible_API Mass<defaulttype::TYPEABSTRACTNAME3dTypes>;
extern template class SOFA_Flexible_API ConstraintCorrection<defaulttype::TYPEABSTRACTNAME3dTypes>;
extern template class SOFA_Flexible_API ProjectiveConstraintSet<defaulttype::TYPEABSTRACTNAME3dTypes>;
#endif
#ifndef SOFA_DOUBLE
extern template class SOFA_Flexible_API ForceField<defaulttype::TYPEABSTRACTNAME3fTypes>;
extern template class SOFA_Flexible_API Mass<defaulttype::TYPEABSTRACTNAME3fTypes>;
extern template class SOFA_Flexible_API ConstraintCorrection<defaulttype::TYPEABSTRACTNAME3fTypes>;
extern template class SOFA_Flexible_API ProjectiveConstraintSet<defaulttype::TYPEABSTRACTNAME3fTypes>;
#endif
#endif


} // namespace behavior



#if defined(SOFA_EXTERN_TEMPLATE) && !defined(FLEXIBLE_COMPILING_CPP)
#ifndef SOFA_FLOAT
extern template class SOFA_Flexible_API Mapping<defaulttype::TYPEABSTRACTNAME3dTypes,defaulttype::Rigid3dTypes>;
#endif
#ifndef SOFA_DOUBLE
extern template class SOFA_Flexible_API Mapping<defaulttype::TYPEABSTRACTNAME3fTypes,defaulttype::Rigid3fTypes>;
#endif
#endif


} // namespace core




} // namespace sofa


#include "ComponentSpecializationsUndef.h"

