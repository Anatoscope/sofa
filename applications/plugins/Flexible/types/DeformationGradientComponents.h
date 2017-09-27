#ifndef FLEXIBLE_DeformationGradientComponents_H
#define FLEXIBLE_DeformationGradientComponents_H

#include <SofaBaseMechanics/MechanicalObject.h>
#include "DeformationGradientTypes.h"


namespace sofa
{


namespace core
{



#if defined(SOFA_EXTERN_TEMPLATE) && !defined(FLEXIBLE_DeformationGradientComponents_CPP)
#ifndef SOFA_FLOAT
extern template class SOFA_Flexible_API State<defaulttype::F331dTypes>;
extern template class SOFA_Flexible_API State<defaulttype::F321dTypes>;
extern template class SOFA_Flexible_API State<defaulttype::F311dTypes>;
extern template class SOFA_Flexible_API State<defaulttype::F332dTypes>;
extern template class SOFA_Flexible_API State<defaulttype::F221dTypes>;
#endif
#ifndef SOFA_DOUBLE
extern template class SOFA_Flexible_API State<defaulttype::F331fTypes>;
extern template class SOFA_Flexible_API State<defaulttype::F321fTypes>;
extern template class SOFA_Flexible_API State<defaulttype::F311fTypes>;
extern template class SOFA_Flexible_API State<defaulttype::F332fTypes>;
extern template class SOFA_Flexible_API State<defaulttype::F221fTypes>;
#endif
#endif


} // namespace core


namespace component
{

namespace container
{

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(FLEXIBLE_DeformationGradientComponents_CPP)
#ifndef SOFA_FLOAT
extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::F331dTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::F331dTypes>;
extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::F332dTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::F332dTypes>;
extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::F321dTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::F321dTypes>;
extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::F311dTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::F311dTypes>;
extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::F221dTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::F221dTypes>;
#endif
#ifndef SOFA_DOUBLE
extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::F331fTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::F331fTypes>;
extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::F332fTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::F332fTypes>;
extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::F321fTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::F321fTypes>;
extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::F311fTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::F311fTypes>;
extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::F221fTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::F221fTypes>;
#endif
#endif

} // namespace container

} // namespace component

} // namespace sofa


#endif
