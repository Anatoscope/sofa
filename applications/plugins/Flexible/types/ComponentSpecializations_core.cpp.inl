

#include <Flexible/config.h>


#include "ComponentSpecializationsDefines.h"



#include <sofa/core/ObjectFactory.h>


#include <sofa/core/State.inl>
#include <sofa/core/behavior/ForceField.inl>
#include <sofa/core/behavior/Mass.inl>
#include <sofa/core/behavior/ProjectiveConstraintSet.inl>
#include <sofa/core/behavior/ConstraintCorrection.inl>
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
    template class SOFA_Flexible_API Mapping< defaulttype::TYPEABSTRACTNAME3dTypes, defaulttype::TYPEABSTRACTNAME3dTypes >;
    template class SOFA_Flexible_API Mapping< defaulttype::TYPEABSTRACTNAME3dTypes, defaulttype::ExtVec3fTypes >;
    template class SOFA_Flexible_API Mapping< defaulttype::TYPEABSTRACTNAME3dTypes, defaulttype::ExtVec3dTypes >;
    template class SOFA_Flexible_API MultiMapping< defaulttype::TYPEABSTRACTNAME3dTypes, defaulttype::TYPEABSTRACTNAME3dTypes >;
#endif
#ifndef SOFA_DOUBLE
    template class SOFA_Flexible_API State< defaulttype::TYPEABSTRACTNAME3fTypes >;
    template class SOFA_Flexible_API Mapping< defaulttype::TYPEABSTRACTNAME3fTypes, defaulttype::Vec3fTypes >;
    template class SOFA_Flexible_API Mapping< defaulttype::TYPEABSTRACTNAME3fTypes, defaulttype::Rigid3fTypes >;
    template class SOFA_Flexible_API Mapping< defaulttype::TYPEABSTRACTNAME3fTypes, defaulttype::TYPEABSTRACTNAME3fTypes >;
    template class SOFA_Flexible_API Mapping< defaulttype::TYPEABSTRACTNAME3fTypes, defaulttype::ExtVec3fTypes >;
    template class SOFA_Flexible_API MultiMapping< defaulttype::TYPEABSTRACTNAME3fTypes, defaulttype::TYPEABSTRACTNAME3fTypes >;
#endif


#ifndef SOFA_FLOAT
#ifndef SOFA_DOUBLE

// cross-terms
template class SOFA_Flexible_API Mapping< defaulttype::TYPEABSTRACTNAME3fTypes, defaulttype::Vec3dTypes >;
template class SOFA_Flexible_API Mapping< defaulttype::TYPEABSTRACTNAME3fTypes, defaulttype::ExtVec3dTypes >;

template class SOFA_Flexible_API Mapping< defaulttype::TYPEABSTRACTNAME3dTypes, defaulttype::Vec3fTypes >;


template class SOFA_Flexible_API Mapping< defaulttype::TYPEABSTRACTNAME3fTypes, 
                                          defaulttype::TYPEABSTRACTNAME3dTypes >;
template class SOFA_Flexible_API Mapping< defaulttype::TYPEABSTRACTNAME3dTypes, 
                                          defaulttype::TYPEABSTRACTNAME3fTypes >;
#endif
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



} // namespace sofa


#include "ComponentSpecializationsUndef.h"
