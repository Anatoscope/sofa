/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2017 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef FLEXIBLE_StrainComponents_H
#define FLEXIBLE_StrainComponents_H

#include "StrainTypes.h"
#include <SofaBaseMechanics/MechanicalObject.h>


namespace sofa
{


namespace core
{



#if defined(SOFA_EXTERN_TEMPLATE) && !defined(FLEXIBLE_StrainComponents_CPP)
#ifndef SOFA_FLOAT
extern template class SOFA_Flexible_API State<defaulttype::E331dTypes>;
extern template class SOFA_Flexible_API State<defaulttype::E321dTypes>;
extern template class SOFA_Flexible_API State<defaulttype::E311dTypes>;
extern template class SOFA_Flexible_API State<defaulttype::E332dTypes>;
extern template class SOFA_Flexible_API State<defaulttype::E333dTypes>;
extern template class SOFA_Flexible_API State<defaulttype::E221dTypes>;
extern template class SOFA_Flexible_API State<defaulttype::I331dTypes>;
extern template class SOFA_Flexible_API State<defaulttype::U331dTypes>;
extern template class SOFA_Flexible_API State<defaulttype::U321dTypes>;
#endif
#ifndef SOFA_DOUBLE
extern template class SOFA_Flexible_API State<defaulttype::E331fTypes>;
extern template class SOFA_Flexible_API State<defaulttype::E321fTypes>;
extern template class SOFA_Flexible_API State<defaulttype::E311fTypes>;
extern template class SOFA_Flexible_API State<defaulttype::E332fTypes>;
extern template class SOFA_Flexible_API State<defaulttype::E333fTypes>;
extern template class SOFA_Flexible_API State<defaulttype::E221fTypes>;
extern template class SOFA_Flexible_API State<defaulttype::I331fTypes>;
extern template class SOFA_Flexible_API State<defaulttype::U331fTypes>;
extern template class SOFA_Flexible_API State<defaulttype::U321fTypes>;
#endif
#endif


} // namespace core


// ==========================================================================
// Mechanical Object

namespace component
{

namespace container
{

#if defined(SOFA_EXTERN_TEMPLATE) && !defined(FLEXIBLE_StrainComponents_CPP)
#ifndef SOFA_FLOAT
//extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::E331dTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::E331dTypes>;
//extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::E332dTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::E332dTypes>;
//extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::E333dTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::E333dTypes>;
//extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::E321dTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::E321dTypes>;
//extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::E311dTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::E311dTypes>;
//extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::E221dTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::E221dTypes>;


//extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::I331dTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::I331dTypes>;

//extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::U331dTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::U331dTypes>;
//extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::U321dTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::U321dTypes>;


#endif
#ifndef SOFA_DOUBLE
//extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::E331fTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::E331fTypes>;
//extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::E332fTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::E332fTypes>;
//extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::E333fTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::E333fTypes>;
//extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::E321fTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::E321fTypes>;
//extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::E311fTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::E311fTypes>;
//extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::E221fTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::E221fTypes>;

//extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::I331fTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::I331fTypes>;

//extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::U331fTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::U331fTypes>;
//extern template class SOFA_Flexible_API MechanicalObjectInternalData<defaulttype::U321fTypes>;
extern template class SOFA_Flexible_API MechanicalObject<defaulttype::U321fTypes>;
#endif
#endif







} // namespace container

} // namespace component



} // namespace sofa


#endif
