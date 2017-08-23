

#include <Flexible/config.h>


#include "ComponentSpecializationsDefines.h"

#include <sofa/core/ObjectFactory.h>

#include <SofaBaseMechanics/IdentityMapping.inl>
#include <SofaMiscMapping/SubsetMultiMapping.inl>


// note: if you lack instantiation of core classes, put them in
// ComponentSpecializations_core.cpp.inl

namespace sofa
{

namespace component
{

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


} // namespace component


} // namespace sofa


#include "ComponentSpecializationsUndef.h"
